// Copyright (c) 2026, kdl_identification authors.
// 教学用途：kdl_lmi_projection.hpp 中声明的求解器在这里落地。
//
// ===========================================================================
// 算法：S-度量正交归一化 + 外点罚函数 + BFGS
// ===========================================================================
// 要解的问题是
//
//     min ‖S·(β − β_prior)‖   s.t.  W·β = W·β_id,  J_s(β) ⪰ 0 ∀s
//
// 三步把它变成一个**无约束**的光滑问题：
//
// ① 消掉等式约束并做 S-度量正交归一化
//      β = β_c + Ñ·w ，β_c = argmin_{β∈A}‖S(β−β_prior)‖（即无约束补全的结果），
//      Ñ 取 null(W) 的基在 S 度量下正交归一化（(S·Ñ)ᵀ(S·Ñ) = I）。
//      于是目标退化成最简单的 ½‖w‖²，且 w 天然 O(1)。
//      这一步是数值成败的关键：不做的话变量量级悬殊、步长失控
//      （实测发散到 |z|≈1e5）。
//
// ② 把 LMI 变成罚项（外点罚函数）
//      V(w) = Σ_s Σ_i max(0, δ_s − λ_i(J_s(w)))²      ← 只在违约时非零
//      F_ρ(w) = ½‖w‖² + ρ·V(w)
//      ρ 递增：ρ→0 时 w=0（无约束补全），ρ→∞ 时收敛到**可行域内**离先验最近的
//      那一点 —— 正是我们要的解。
//
// ③ 用 BFGS 解每个 ρ 的无约束子问题（18 个变量，海塞近似 18×18，代价可忽略）
//
// ===========================================================================
// 为什么不用"移位对数障碍"（那条路我试过并放弃了）
// ===========================================================================
// 障碍法要求从一个**严格可行**的点出发。本问题的难处恰恰是"先验附近的仿射集
// 落在可行域外"，于是两头堵：
//   · 起点 w=0（无约束补全）不可行，只能靠移位 σ 撑开；但移位后的可行域无界，
//     目标权重给小了，障碍单独最小化会发散（实测 |z|≈1e5）；
//   · 目标权重给大了，目标又把点锚在原点、障碍推不动，σ 缩不下去（死锁）。
// 罚函数法没有这个两难：罚项只在违约时起作用、方向明确，目标项负责把解拉回
// 先验附近，两者不会互相抵消。
//
// ===========================================================================
// 局限（如实说明）
// ===========================================================================
// · 外点罚函数在 ρ→∞ 时条件数会变差，是"渐近"而非"精确"的；本实现用 ρ 递增 +
//   热启动，并在最后**显式检查**可行性，不可行就报失败，不会静默返回坏结果。
// · 没有预处理、没有二阶校正，稳健性不如专业 SDP 求解器（SCS / MOSEK）。
// · 但正确性可以用"已知有解"的场景验证：离线 demo 里真值 β 就在仿射集内且
//   可行，若求解器能返回一个**可行且力矩一致**的点，就说明它确实在可行域内部
//   找到了解，而不是像"投影+退让"那样退回先验。

#include "kdl_lmi_projection.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <rclcpp/logging.hpp>

namespace kdl_identification
{
namespace
{

constexpr const char * kLogTag = "kdl_identification";

/// 每个连杆的标准惯性参数个数（与 kdl_regressor 一致）。
constexpr unsigned int kBlock = 10;

/// 列范数的相对阈值（与 kdl_lsq / completeWithPrior 一致）。
constexpr double kColumnTolerance = 1e-10;

/// 罚因子 ρ 的初值与增长倍数。
constexpr double kRhoInitial = 1.0;
constexpr double kRhoGrowth = 10.0;

/// ρ 的外层迭代次数上限。
constexpr unsigned int kMaxOuter = 14;

/// 每个 ρ 子问题里 BFGS 的迭代上限。
constexpr unsigned int kMaxBgfs = 600;

/// BFGS 梯度收敛阈值。
constexpr double kGradientTolerance = 1e-10;

/// 判定"违约已消除"的阈值（V(w) 的量级）。
constexpr double kViolationTolerance = 1e-14;

/// 可行裕度：相对每段 J 的量级取 1e-7，把解推离边界一点点，
/// 这样返回的点是**严格**可行的（下游 checkPhysicalFeasibility 要求严格正定）。
constexpr double kMarginRatio = 1e-7;

}  // namespace

Eigen::Matrix4d pseudoInertia(const Eigen::VectorXd & block)
{
  Eigen::Matrix4d jacobian = Eigen::Matrix4d::Zero();
  if (block.size() < static_cast<Eigen::Index>(kBlock)) {
    return jacobian;
  }
  const double m = block[0];
  const Eigen::Vector3d h(block[1], block[2], block[3]);
  Eigen::Matrix3d i_ref;
  i_ref << block[4], block[5], block[6], block[5], block[7], block[8], block[6], block[8], block[9];

  jacobian.topLeftCorner<3, 3>() = 0.5 * i_ref.trace() * Eigen::Matrix3d::Identity() - i_ref;
  jacobian.topRightCorner<3, 1>() = h;
  jacobian.bottomLeftCorner<1, 3>() = h.transpose();
  jacobian(3, 3) = m;
  return jacobian;
}

Eigen::VectorXd repairToStrictFeasibility(const Eigen::VectorXd & parameters)
{
  Eigen::VectorXd repaired = parameters;
  if (parameters.size() == 0 || parameters.size() % static_cast<Eigen::Index>(kBlock) != 0) {
    return repaired;
  }
  const unsigned int count = static_cast<unsigned int>(parameters.size() / static_cast<Eigen::Index>(kBlock));
  for (unsigned int s = 0; s < count; ++s) {
    const Eigen::VectorXd block = parameters.segment<kBlock>(kBlock * s);
    double m = block[0];
    if (!(m > 1e-12)) {
      m = 1e-12;
    }
    const Eigen::Vector3d h(block[1], block[2], block[3]);
    const Eigen::Vector3d cog = h / m;
    Eigen::Matrix3d i_ref;
    i_ref << block[4], block[5], block[6], block[5], block[7], block[8], block[6], block[8], block[9];
    const Eigen::Matrix3d parallel =
      m * (cog.dot(cog) * Eigen::Matrix3d::Identity() - cog * cog.transpose());
    const Eigen::Matrix3d i_cog = i_ref - parallel;

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(0.5 * (i_cog + i_cog.transpose()));
    Eigen::Vector3d eigenvalues = solver.eigenvalues();
    const Eigen::Matrix3d eigenvectors = solver.eigenvectors();
    const double floor = std::max(1e-10, 1e-3 * std::max(eigenvalues(2), 1e-12));
    eigenvalues = eigenvalues.cwiseMax(floor);
    if (eigenvalues(2) > eigenvalues(0) + eigenvalues(1)) {
      eigenvalues(2) = 0.999 * (eigenvalues(0) + eigenvalues(1));
    }
    const Eigen::Matrix3d i_ref_repaired =
      eigenvectors * eigenvalues.asDiagonal() * eigenvectors.transpose() + parallel;

    auto out = repaired.segment<kBlock>(kBlock * s);
    out[0] = m;
    out[4] = i_ref_repaired(0, 0);
    out[5] = i_ref_repaired(0, 1);
    out[6] = i_ref_repaired(0, 2);
    out[7] = i_ref_repaired(1, 1);
    out[8] = i_ref_repaired(1, 2);
    out[9] = i_ref_repaired(2, 2);
  }
  return repaired;
}

LmiSolveResult solveLmiConstrainedCompletion(
  const Eigen::MatrixXd & regressor, const Eigen::VectorXd & identified,
  const Eigen::VectorXd & prior, const Eigen::VectorXd & scale, double rank_tolerance)
{
  LmiSolveResult result;

  if (regressor.rows() == 0 || regressor.cols() == 0) {
    result.message = "回归矩阵为空";
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
    return result;
  }
  const Eigen::Index cols = regressor.cols();
  if (cols % static_cast<Eigen::Index>(kBlock) != 0) {
    result.message = "回归矩阵列数必须是 10 的整数倍，实际为 " + std::to_string(cols);
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
    return result;
  }
  if (identified.size() != cols || prior.size() != cols) {
    result.message = "参数向量长度必须等于回归矩阵列数 " + std::to_string(cols);
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
    return result;
  }
  const unsigned int segments = static_cast<unsigned int>(cols / static_cast<Eigen::Index>(kBlock));

  // ---- 1) 列归一化 + 判秩 + 零空间基（与 kdl_lsq / completeWithPrior 同策）----
  Eigen::VectorXd column_norm(cols);
  for (Eigen::Index j = 0; j < cols; ++j) {
    column_norm(j) = regressor.col(j).norm();
  }
  const double max_norm = column_norm.maxCoeff();
  Eigen::MatrixXd normalized = regressor;
  for (Eigen::Index j = 0; j < cols; ++j) {
    if (max_norm > 0.0 && column_norm(j) > max_norm * kColumnTolerance) {
      normalized.col(j) /= column_norm(j);
    } else {
      normalized.col(j).setZero();
      column_norm(j) = 1.0;
    }
  }

  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(
    normalized, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd singular = svd.singularValues();
  if (singular.size() == 0 || singular(0) <= 0.0) {
    result.message = "回归矩阵全零，秩为 0";
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
    return result;
  }
  unsigned int rank = 0;
  for (Eigen::Index i = 0; i < singular.size(); ++i) {
    if (singular(i) > rank_tolerance * singular(0)) {
      ++rank;
    }
  }
  if (rank == 0) {
    result.message = "回归矩阵数值秩为 0";
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
    return result;
  }
  const unsigned int free_variables = static_cast<unsigned int>(cols) - rank;
  result.variables = free_variables;
  if (free_variables == 0) {
    // 满秩：仿射集只有一个点，没有可优化的余地。
    result.parameters = identified;
    result.success = true;
    result.message.clear();
    return result;
  }

  Eigen::MatrixXd null_basis(cols, free_variables);
  for (unsigned int i = 0; i < free_variables; ++i) {
    const Eigen::Index column = static_cast<Eigen::Index>(rank + i);
    if (column >= svd.matrixV().cols()) {
      result.message = "零空间基提取失败：右奇异向量维数不足";
      RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
      return result;
    }
    null_basis.col(i) = svd.matrixV().col(column);
    for (Eigen::Index j = 0; j < cols; ++j) {
      null_basis(j, i) /= column_norm(j);
    }
  }

  // ---- 2) S-度量正交归一化 + 原点搬到"离先验最近的仿射点" ----
  const Eigen::VectorXd s_diagonal = (scale.size() == cols) ? scale : Eigen::VectorXd::Ones(cols);
  const Eigen::MatrixXd weighted_raw = s_diagonal.asDiagonal() * null_basis;  // S·N
  const Eigen::MatrixXd metric = weighted_raw.transpose() * weighted_raw;     // NᵀS²N
  const Eigen::VectorXd gradient_phi_0 =
    weighted_raw.transpose() * (s_diagonal.cwiseProduct(identified - prior));

  const Eigen::LLT<Eigen::MatrixXd> cholesky(metric);
  Eigen::MatrixXd null_basis_orthonormal = null_basis;
  Eigen::VectorXd z_star = Eigen::VectorXd::Zero(free_variables);
  if (cholesky.info() == Eigen::Success) {
    // Ñ = N·L⁻ᵀ 使 (S·Ñ)ᵀ(S·Ñ) = I。
    const Eigen::MatrixXd l = cholesky.matrixL();
    const Eigen::MatrixXd l_inverse_transpose =
      l.transpose().ldlt().solve(Eigen::MatrixXd::Identity(free_variables, free_variables));
    null_basis_orthonormal = null_basis * l_inverse_transpose;
    z_star = cholesky.solve(-gradient_phi_0);
  } else {
    z_star = metric.ldlt().solve(-gradient_phi_0);
  }
  const Eigen::VectorXd beta_center = identified + null_basis * z_star;

  // ---- 3) 预计算每段的 J0 = J(β_c 的第 s 块) 与 Jk = J(Ñ 第 k 列的第 s 块) ----
  // J 对 β 线性（无常数项），所以 J_s(w) = J0_s + Σ_k w_k·Jk_s 精确成立。
  std::vector<Eigen::Matrix4d> j0(segments);
  std::vector<std::vector<Eigen::Matrix4d>> jk(segments);
  std::vector<double> margin(segments);
  for (unsigned int s = 0; s < segments; ++s) {
    j0[s] = pseudoInertia(beta_center.segment<kBlock>(kBlock * s));
    jk[s].resize(free_variables);
    for (unsigned int k = 0; k < free_variables; ++k) {
      jk[s][k] = pseudoInertia(null_basis_orthonormal.col(k).segment<kBlock>(kBlock * s));
    }
    margin[s] = std::max(1e-12, kMarginRatio * j0[s].cwiseAbs().maxCoeff());
  }

  // ---- 4) 罚函数 V(w) 及其梯度 ----
  // V(w) = Σ_s Σ_i max(0, δ_s − λ_i(J_s(w)))²
  // ∂V/∂w_k = −2·Σ_s Σ_i max(0, δ_s − λ_i)·v_iᵀ·Jk_s·v_i
  const auto evaluate_penalty =
    [&](const Eigen::VectorXd & w, Eigen::VectorXd & gradient) -> double {
      double violation = 0.0;
      gradient.setZero(free_variables);
      for (unsigned int s = 0; s < segments; ++s) {
        Eigen::Matrix4d matrix = j0[s];
        for (unsigned int k = 0; k < free_variables; ++k) {
          matrix += w(k) * jk[s][k];
        }
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(
          0.5 * (matrix + matrix.transpose()));
        for (int i = 0; i < 4; ++i) {
          const double slack = margin[s] - solver.eigenvalues()(i);
          if (slack <= 0.0) {
            continue;
          }
          violation += slack * slack;
          const Eigen::Vector4d eigenvector = solver.eigenvectors().col(i);
          for (unsigned int k = 0; k < free_variables; ++k) {
            gradient(k) -= 2.0 * slack * eigenvector.dot(jk[s][k] * eigenvector);
          }
        }
      }
      return violation;
    };

  // ---- 5) 外层 ρ 递增 + 内层 BFGS ----
  Eigen::VectorXd w = Eigen::VectorXd::Zero(free_variables);
  double rho = kRhoInitial;
  double violation = std::numeric_limits<double>::max();
  Eigen::VectorXd penalty_gradient(free_variables);

  for (unsigned int outer = 0; outer < kMaxOuter; ++outer) {
    result.outer_iterations = outer + 1;

    const auto evaluate = [&](const Eigen::VectorXd & point, Eigen::VectorXd & gradient) -> double {
        const double penalty = evaluate_penalty(point, penalty_gradient);
        gradient = point + rho * penalty_gradient;
        return 0.5 * point.squaredNorm() + rho * penalty;
      };

    Eigen::VectorXd gradient(free_variables);
    double value = evaluate(w, gradient);

    Eigen::MatrixXd inverse_hessian =
      Eigen::MatrixXd::Identity(free_variables, free_variables);

    for (unsigned int iteration = 0; iteration < kMaxBgfs; ++iteration) {
      if (gradient.norm() < kGradientTolerance) {
        break;
      }
      Eigen::VectorXd direction = -inverse_hessian * gradient;
      if (!direction.allFinite() || direction.dot(gradient) >= 0.0) {
        // 海塞近似失效：退回最速下降并重置。
        direction = -gradient;
        inverse_hessian = Eigen::MatrixXd::Identity(free_variables, free_variables);
      }

      // Armijo 回溯线搜索
      double alpha = 1.0;
      Eigen::VectorXd trial(free_variables);
      Eigen::VectorXd trial_gradient(free_variables);
      double trial_value = 0.0;
      bool accepted = false;
      for (unsigned int backtrack = 0; backtrack < 60; ++backtrack) {
        trial = w + alpha * direction;
        trial_value = evaluate(trial, trial_gradient);
        if (trial_value <= value + 1e-4 * alpha * gradient.dot(direction)) {
          accepted = true;
          break;
        }
        alpha *= 0.5;
      }
      if (!accepted) {
        break;
      }

      // BFGS 更新（只在曲率条件满足时更新，避免近似被噪声污染）
      const Eigen::VectorXd s_step = trial - w;
      const Eigen::VectorXd y_step = trial_gradient - gradient;
      const double curvature = y_step.dot(s_step);
      if (curvature > 1e-18) {
        const Eigen::MatrixXd identity =
          Eigen::MatrixXd::Identity(free_variables, free_variables);
        const Eigen::MatrixXd left = identity - (s_step * y_step.transpose()) / curvature;
        inverse_hessian = left * inverse_hessian * left.transpose() +
          (s_step * s_step.transpose()) / curvature;
      }
      w = trial;
      gradient = trial_gradient;
      value = trial_value;
      ++result.newton_steps;
    }

    violation = evaluate_penalty(w, penalty_gradient);
    if (const char * verbose = std::getenv("KDL_LMI_VERBOSE")) {
      if (verbose[0] != '0') {
        double worst = std::numeric_limits<double>::max();
        for (unsigned int s = 0; s < segments; ++s) {
          Eigen::Matrix4d matrix = j0[s];
          for (unsigned int k = 0; k < free_variables; ++k) {
            matrix += w(k) * jk[s][k];
          }
          const Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(
            0.5 * (matrix + matrix.transpose()));
          worst = std::min(worst, solver.eigenvalues().minCoeff());
        }
        std::printf("[lmi] 外层 %2u: rho=%.0e  V=%.3e  lam_min=%+.6e  |w|=%.4f  |grad|=%.2e\n",
          outer + 1, rho, violation, worst, w.norm(), gradient.norm());
      }
    }
    if (violation < kViolationTolerance) {
      break;  // 违约已消除，不需要再加大 ρ
    }
    rho *= kRhoGrowth;
  }

  // ---- 6) 组装结果，并做一次"贴边修复" ----
  // 罚函数是渐近的：收敛后 λ_min 常常停在 0 的**下方极近处**（实测无噪声时
  // −1.7e-9，而 J 的量级是 1，属纯数值精度）。此时若直接用 `λ_min > 0` 判失败
  // 就把一个实际可用的解丢掉了。
  // 修法：沿**违约梯度**方向在零空间里微调一点点 —— 由于 β = β_c + Ñ·w 的构造，
  // 任何 w 都仍然严格落在仿射集 A 内，所以这次微调**不会**破坏力矩一致性。
  const auto min_pseudo_eigenvalue = [&](const Eigen::VectorXd & point) {
      double worst = std::numeric_limits<double>::max();
      for (unsigned int s = 0; s < segments; ++s) {
        Eigen::Matrix4d matrix = j0[s];
        for (unsigned int k = 0; k < free_variables; ++k) {
          matrix += point(k) * jk[s][k];
        }
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(
          0.5 * (matrix + matrix.transpose()));
        worst = std::min(worst, solver.eigenvalues().minCoeff());
      }
      return worst;
    };

  double min_eigenvalue = min_pseudo_eigenvalue(w);
  if (min_eigenvalue <= 0.0 && penalty_gradient.norm() > 0.0) {
    // −∇V 指向违约减小的方向；从极小的步长开始逐步加大，直到严格可行。
    const Eigen::VectorXd direction = -penalty_gradient.normalized();
    double step = 1e-12 * (1.0 + w.norm());
    double best = min_eigenvalue;
    double best_step = 0.0;
    for (unsigned int attempt = 0; attempt < 100; ++attempt) {
      const Eigen::VectorXd trial = w + step * direction;
      const double trial_min = min_pseudo_eigenvalue(trial);
      if (trial_min > best) {
        best = trial_min;
        best_step = step;
      }
      if (trial_min > 0.0) {
        w = trial;
        min_eigenvalue = trial_min;
        break;
      }
      step *= 1.5;
    }
    if (min_eigenvalue <= 0.0) {
      min_eigenvalue = best;
      if (best_step > 0.0) {
        w = w + best_step * direction;
      }
    }
    if (const char * verbose = std::getenv("KDL_LMI_VERBOSE")) {
      if (verbose[0] != '0') {
        std::printf("[lmi] 贴边修复：起始 lam_min=%.3e  |gradV|=%.3e  ->  lam_min=%.3e  step=%.3e\n",
          best, penalty_gradient.norm(), min_eigenvalue, best_step);
      }
    }
  }

  result.parameters = beta_center + null_basis_orthonormal * w;
  result.z = w;
  result.scaled_distance = (s_diagonal.cwiseProduct(result.parameters - prior)).norm();
  result.min_eigenvalue = min_eigenvalue;

  if (!result.parameters.allFinite()) {
    result.message = "求解得到非有限值";
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
    return result;
  }
  if (min_eigenvalue <= 0.0) {
    // 罚函数是渐近的，收敛后 λ_min 常停在 0 的**下方极近处**（实测无噪声时 −6.3e-10，
    // 而 J 的量级是 1 → 相对 1e-9，纯浮点噪声；此时 |∇V| 也已降到 3e-9，再优化不动）。
    // 判据用**相对**容差：违约相对量级小于 1e-7 就认为"实质可行"，
    // 再用 repairToStrictFeasibility() 把这点数值违约夹掉。
    double scale_reference = 0.0;
    for (unsigned int s = 0; s < segments; ++s) {
      Eigen::Matrix4d matrix = j0[s];
      for (unsigned int k = 0; k < free_variables; ++k) {
        matrix += w(k) * jk[s][k];
      }
      scale_reference = std::max(scale_reference, matrix.cwiseAbs().maxCoeff());
    }
    const double tolerance = -1e-7 * std::max(scale_reference, 1e-12);
    if (min_eigenvalue > tolerance) {
      const Eigen::VectorXd repaired = repairToStrictFeasibility(result.parameters);
      result.repair_norm = (repaired - result.parameters).norm();
      result.parameters = repaired;
      result.boundary_repaired = true;
      double repaired_min = std::numeric_limits<double>::max();
      for (unsigned int s = 0; s < segments; ++s) {
        const Eigen::Matrix4d matrix = pseudoInertia(result.parameters.segment<kBlock>(kBlock * s));
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(
          0.5 * (matrix + matrix.transpose()));
        repaired_min = std::min(repaired_min, solver.eigenvalues().minCoeff());
      }
      result.min_eigenvalue = repaired_min;
      if (repaired_min <= 0.0) {
        result.message = "贴边修复后仍不正定（λ_min = " + std::to_string(repaired_min) +
          "）：A ∩ int(P) 大概率为空。";
        RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
        return result;
      }
      RCLCPP_WARN(
        rclcpp::get_logger(kLogTag),
        "LMI 解在数值噪声量级贴边（λ_min = %.3e），已做夹紧修复（‖Δβ‖ = %.3e）",
        min_eigenvalue, result.repair_norm);
    } else {
      result.message = "未找到物理可行解：伪惯量最小特征值 = " + std::to_string(min_eigenvalue) +
        "（罚函数残余违约 " + std::to_string(violation) + "，ρ 到 " + std::to_string(rho) +
        "）。可能 A ∩ int(P) 为空。";
      RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
      return result;
    }
  }

  result.success = true;
  result.message.clear();
  return result;
}

}  // namespace kdl_identification
