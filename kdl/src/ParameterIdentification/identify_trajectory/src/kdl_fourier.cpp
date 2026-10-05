// Copyright (c) 2026, kdl_identification authors.
// 教学用途：kdl_fourier.hpp 中声明的函数在这里落地。
//
// 文件顺序与头文件一一对应：
//   一、傅里叶轨迹            —— 采样公式（纯代数）
//   二、约束与校验            —— 采样求峰值 + "缩放到贴住约束"
//   三、可观测性              —— 列归一化回归矩阵的奇异值
//   四、激励轨迹优化          —— 随机重启 + 局部搜索
//
// 每一处用到"回归矩阵"的地方都通过预先构造好的 RegressorBuilder 完成：
// 它把 10·段数 + 1 条扰动链的建链开销只付一次，使得几千次目标函数评估
// 仍然能在几秒内跑完。

#include "kdl_fourier.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/QR>
#include <Eigen/SVD>
#include <rclcpp/logging.hpp>

namespace kdl_identification
{
namespace
{

/// 所有日志统一前缀，方便在终端里一眼看出是谁打的。
constexpr const char * kLogTag = "kdl_identification";

/// 判定"比值是否越界"的容差：1 + 1e-9。
constexpr double kRatioTolerance = 1e-9;

/// 缩放时留的余量：峰值靠采样求得，可能略低于真实峰值，留 1% 防止越界。
constexpr double kScalingSafety = 0.99;

/// 列范数的相对阈值：低于"最大列范数 × 该比例"的列视为恒零列（不可辨识）。
/// 与 kdl_lsq 里 LeastSquaresOptions::column_tolerance 的默认值保持一致。
constexpr double kColumnTolerance = 1e-10;

}  // namespace

// ---------------------------------------------------------------------------
// 一、傅里叶轨迹
// ---------------------------------------------------------------------------

FourierTrajectory::FourierTrajectory(
  unsigned int harmonics, double period, const KDL::JntArray & offset,
  const Eigen::MatrixXd & sine_amplitudes, const Eigen::MatrixXd & cosine_amplitudes)
: harmonics_(harmonics), period_(period), offset_(offset)
{
  const unsigned int n = offset.rows();
  if (harmonics == 0 || period <= 0.0 || n == 0) {
    return;
  }

  // 幅值矩阵允许为空（等价于全 0），否则必须正好是 n × H。
  const bool sine_ok = (sine_amplitudes.size() == 0) ||
    (sine_amplitudes.rows() == static_cast<Eigen::Index>(n) &&
    sine_amplitudes.cols() == static_cast<Eigen::Index>(harmonics));
  const bool cosine_ok = (cosine_amplitudes.size() == 0) ||
    (cosine_amplitudes.rows() == static_cast<Eigen::Index>(n) &&
    cosine_amplitudes.cols() == static_cast<Eigen::Index>(harmonics));
  if (!sine_ok || !cosine_ok) {
    return;
  }

  joints_ = n;
  sine_amplitudes_ = Eigen::MatrixXd::Zero(n, harmonics);
  cosine_amplitudes_ = Eigen::MatrixXd::Zero(n, harmonics);
  if (sine_amplitudes.size() > 0) {
    sine_amplitudes_ = sine_amplitudes;
  }
  if (cosine_amplitudes.size() > 0) {
    cosine_amplitudes_ = cosine_amplitudes;
  }

  is_valid_ = true;
}

bool FourierTrajectory::valid() const
{
  return is_valid_;
}

bool FourierTrajectory::sample(
  double t, KDL::JntArray & q, KDL::JntArray & qdot, KDL::JntArray & qddot) const
{
  if (!is_valid_) {
    return false;
  }

  q = KDL::JntArray(joints_);
  qdot = KDL::JntArray(joints_);
  qddot = KDL::JntArray(joints_);

  const double omega = baseFrequency();
  for (unsigned int j = 0; j < joints_; ++j) {
    double position = offset_(j);
    double velocity = 0.0;
    double acceleration = 0.0;

    for (unsigned int l = 1; l <= harmonics_; ++l) {
      const double l_omega = static_cast<double>(l) * omega;
      const double phase = l_omega * t;
      const double sin_phase = std::sin(phase);
      const double cos_phase = std::cos(phase);
      const double a = sine_amplitudes_(j, l - 1);
      const double b = cosine_amplitudes_(j, l - 1);

      // q  = q0 + Σ [ a/(lω)·sin(lωt) − b/(lω)·cos(lωt) ]
      position += (a * sin_phase - b * cos_phase) / l_omega;
      // q̇ = Σ [ a·cos(lωt) + b·sin(lωt) ]
      velocity += a * cos_phase + b * sin_phase;
      // q̈ = Σ lω·[ −a·sin(lωt) + b·cos(lωt) ]
      acceleration += l_omega * (-a * sin_phase + b * cos_phase);
    }

    q(j) = position;
    qdot(j) = velocity;
    qddot(j) = acceleration;
  }

  return true;
}

bool FourierTrajectory::samplePeriod(
  unsigned int samples, std::vector<RegressorSample> & out) const
{
  if (!is_valid_ || samples == 0) {
    return false;
  }

  out.clear();
  out.reserve(samples);
  for (unsigned int k = 0; k < samples; ++k) {
    const double t = period_ * static_cast<double>(k) / static_cast<double>(samples);
    // 注意局部变量不能叫 sample，否则会遮蔽成员函数 sample()。
    RegressorSample point;
    if (!sample(t, point.q, point.qdot, point.qddot)) {
      return false;
    }
    out.push_back(std::move(point));
  }
  return true;
}

FourierTrajectory FourierTrajectory::scaled(double factor) const
{
  return FourierTrajectory(
    harmonics_, period_, offset_, factor * sine_amplitudes_, factor * cosine_amplitudes_);
}

// ---------------------------------------------------------------------------
// 二、约束与校验
// ---------------------------------------------------------------------------

FourierValidation validateFourierTrajectory(
  const FourierTrajectory & trajectory, const FourierLimits & limits,
  unsigned int samples_per_period)
{
  FourierValidation result;
  const unsigned int n = trajectory.joints();
  if (!trajectory.valid() || samples_per_period == 0) {
    result.message = "轨迹无效或采样点数为 0，无法校验";
    return result;
  }

  const bool check_position = limits.checkPosition(n);
  const bool check_velocity = limits.checkVelocity(n);
  const bool check_acceleration = limits.checkAcceleration(n);
  if (!check_position && !check_velocity && !check_acceleration) {
    result.valid = true;
    result.message.clear();
    return result;
  }

  // 每个类别各自的"最坏关节"，最后用于给出可读的越界说明。
  unsigned int position_joint = 0;
  unsigned int velocity_joint = 0;
  unsigned int acceleration_joint = 0;

  std::vector<RegressorSample> samples;
  trajectory.samplePeriod(samples_per_period, samples);

  for (const RegressorSample & sample : samples) {
    for (unsigned int j = 0; j < n; ++j) {
      if (check_position) {
        const double half = 0.5 * (limits.q_max(j) - limits.q_min(j));
        const double middle = 0.5 * (limits.q_max(j) + limits.q_min(j));
        if (half > 0.0) {
          const double ratio = std::abs(sample.q(j) - middle) / half;
          if (ratio > result.position_ratio) {
            result.position_ratio = ratio;
            position_joint = j;
          }
        }
      }
      if (check_velocity && limits.qdot_max(j) > 0.0) {
        const double ratio = std::abs(sample.qdot(j)) / limits.qdot_max(j);
        if (ratio > result.velocity_ratio) {
          result.velocity_ratio = ratio;
          velocity_joint = j;
        }
      }
      if (check_acceleration && limits.qddot_max(j) > 0.0) {
        const double ratio = std::abs(sample.qddot(j)) / limits.qddot_max(j);
        if (ratio > result.acceleration_ratio) {
          result.acceleration_ratio = ratio;
          acceleration_joint = j;
        }
      }
    }
  }

  result.valid = result.position_ratio <= 1.0 + kRatioTolerance &&
    result.velocity_ratio <= 1.0 + kRatioTolerance &&
    result.acceleration_ratio <= 1.0 + kRatioTolerance;

  // 报出"比值最大的那一项"的关节，便于定位。
  const double worst = std::max({result.position_ratio, result.velocity_ratio, result.acceleration_ratio});
  if (worst == result.position_ratio) {
    result.worst_joint = position_joint;
  } else if (worst == result.velocity_ratio) {
    result.worst_joint = velocity_joint;
  } else {
    result.worst_joint = acceleration_joint;
  }

  if (!result.valid) {
    result.message = "约束越界：位置 " + std::to_string(result.position_ratio) + " / 速度 " +
      std::to_string(result.velocity_ratio) + " / 加速度 " +
      std::to_string(result.acceleration_ratio) + "（1 为上限，关节 " +
      std::to_string(result.worst_joint) + "）";
  } else {
    result.message.clear();
  }

  return result;
}

FourierScaling scaleToFitLimits(
  const FourierTrajectory & trajectory, const FourierLimits & limits,
  unsigned int samples_per_period)
{
  FourierScaling result;
  if (!trajectory.valid()) {
    result.message = "轨迹无效，无法缩放";
    return result;
  }

  const FourierValidation current = validateFourierTrajectory(trajectory, limits, samples_per_period);
  const double max_ratio =
    std::max({current.position_ratio, current.velocity_ratio, current.acceleration_ratio});

  if (max_ratio <= 0.0) {
    // 没有任何约束或轨迹完全静止：保持原样。
    result.success = true;
    result.scale = 1.0;
    result.validation = current;
    return result;
  }

  // 峰值随幅值线性放大，所以 λ = 安全系数 / max(ratio) 正好把峰值顶到上限略下方。
  const double scale = kScalingSafety / max_ratio;
  result.scale = scale;
  result.validation = validateFourierTrajectory(trajectory.scaled(scale), limits, samples_per_period);
  result.success = true;
  return result;
}

// ---------------------------------------------------------------------------
// 三、可观测性
// ---------------------------------------------------------------------------

ObservabilityReport evaluateObservability(
  const RegressorBuilder & builder, const FourierTrajectory & trajectory,
  unsigned int samples_per_period, double rank_tolerance)
{
  ObservabilityReport report;
  report.total_parameters = builder.parameterCount();

  if (!builder.valid()) {
    report.message = "回归矩阵构造器不可用：" + builder.error();
    return report;
  }

  std::vector<RegressorSample> samples;
  if (!trajectory.samplePeriod(samples_per_period, samples)) {
    report.message = "轨迹采样失败";
    return report;
  }

  const RegressorResult regressor = builder.build(samples);
  if (!regressor.success) {
    report.message = "构建回归矩阵失败：" + regressor.message;
    return report;
  }

  // 列归一化：消除 kg 与 kg·m² 的量纲差异，让 SVD 只反映"方向相关性"。
  // 恒零列必须用**相对**阈值剔除（理由同 kdl_lsq）：否则浮点残渣会被放大成
  // 单位列，把"不可辨识"算成"可辨识"，秩和条件数都会失真。
  Eigen::MatrixXd normalized = regressor.regressor;
  double max_norm = 0.0;
  for (Eigen::Index j = 0; j < normalized.cols(); ++j) {
    max_norm = std::max(max_norm, normalized.col(j).norm());
  }
  for (Eigen::Index j = 0; j < normalized.cols(); ++j) {
    const double norm = normalized.col(j).norm();
    if (max_norm > 0.0 && norm > max_norm * kColumnTolerance) {
      normalized.col(j) /= norm;
    } else {
      normalized.col(j).setZero();
    }
  }

  // 只关心奇异值，而 W 很高很瘦（行数 = 采样点数 × 关节数，列数 = 10·段数）。
  // 直接对 W 做 SVD 很贵；先做一次 QR：W = Q·R，则 WᵀW = RᵀR，
  // 所以 W 的奇异值 == 上三角 R 的奇异值，而 R 只有"列数 × 列数"大小。
  Eigen::MatrixXd triangular;
  {
    const Eigen::HouseholderQR<Eigen::MatrixXd> qr(normalized);
    triangular = qr.matrixQR()
                   .topLeftCorner(normalized.cols(), normalized.cols())
                   .template triangularView<Eigen::Upper>();
  }

  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(triangular);
  const Eigen::VectorXd singular_values = svd.singularValues();
  if (singular_values.size() == 0 || singular_values(0) <= 0.0) {
    report.message = "回归矩阵全零，轨迹没有激励";
    return report;
  }

  const double max_singular = singular_values(0);
  unsigned int rank = 0;
  for (Eigen::Index i = 0; i < singular_values.size(); ++i) {
    if (singular_values(i) > rank_tolerance * max_singular) {
      ++rank;
    }
  }
  if (rank == 0) {
    report.message = "回归矩阵秩为 0，轨迹没有激励";
    return report;
  }

  report.rank = rank;
  report.min_singular_value = singular_values(rank - 1);
  report.condition_number = max_singular / singular_values(rank - 1);
  report.success = true;
  report.message.clear();
  return report;
}

// ---------------------------------------------------------------------------
// 四、激励轨迹优化
// ---------------------------------------------------------------------------

ExcitationResult optimizeFourierExcitation(
  const KDL::Chain & chain, const FourierLimits & limits, const ExcitationOptions & options)
{
  ExcitationResult result;
  const unsigned int n = chain.getNrOfJoints();

  if (n == 0) {
    result.message = "链没有自由度，无法生成激励轨迹";
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
    return result;
  }
  if (!limits.checkPosition(n)) {
    result.message = "必须提供各关节的位置上下限（用来确定激励中点与幅值）";
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
    return result;
  }
  if (options.harmonics == 0 || options.period <= 0.0 || options.samples_per_period == 0) {
    result.message = "选项非法：谐波数/周期/采样点数必须为正";
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
    return result;
  }

  RegressorBuilder builder(chain, options.gravity);
  if (!builder.valid()) {
    result.message = "回归矩阵构造器不可用：" + builder.error();
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
    return result;
  }

  // 激励中心取关节行程中点。
  KDL::JntArray center(n);
  for (unsigned int j = 0; j < n; ++j) {
    center(j) = 0.5 * (limits.q_min(j) + limits.q_max(j));
  }

  const unsigned int harmonics = options.harmonics;
  const double period = options.period;

  std::mt19937 generator(options.random_seed);
  std::uniform_real_distribution<double> unit(-1.0, 1.0);
  std::uniform_int_distribution<unsigned int> joint_pick(0, n - 1);
  std::uniform_int_distribution<unsigned int> harmonic_pick(0, harmonics - 1);
  std::uniform_int_distribution<int> component_pick(0, 1);

  unsigned int evaluations = 0;

  // 约束缩放/校验用更密的采样网格（与目标函数评估解耦）：目标函数可以稀一点，
  // 但"峰值是否越界"必须准，否则最终校验会打脸。
  const unsigned int limit_samples = std::max(options.samples_per_period, 200u);

  // 评估一个候选：缩放到贴住约束，再算可观测性。
  auto evaluate_candidate = [&](
                              const Eigen::MatrixXd & a, const Eigen::MatrixXd & b,
                              FourierTrajectory & out, ObservabilityReport & report,
                              double & scale) -> bool {
      if (evaluations >= options.max_evaluations) {
        return false;
      }
      const FourierTrajectory unit_trajectory(harmonics, period, center, a, b);
      const FourierScaling scaling = scaleToFitLimits(unit_trajectory, limits, limit_samples);
      if (!scaling.success || scaling.scale <= 0.0) {
        return false;
      }
      out = unit_trajectory.scaled(scaling.scale);
      report = evaluateObservability(builder, out, options.samples_per_period, options.rank_tolerance);
      ++evaluations;
      if (!report.success) {
        return false;
      }
      scale = scaling.scale;
      return true;
    };

  // 择优准则：先比可辨识基参数个数，再比最弱方向的信号强度。
  auto is_better = [](unsigned int rank1, double min1, unsigned int rank2, double min2) {
      return rank1 > rank2 || (rank1 == rank2 && min1 > min2);
    };

  bool has_best = false;
  FourierTrajectory best_trajectory;
  ObservabilityReport best_report;
  double best_scale = 0.0;

  for (unsigned int restart = 0; restart < options.restarts; ++restart) {
    if (evaluations >= options.max_evaluations) {
      break;
    }

    // 形状：幅值随谐波次数 1/l 衰减（高次谐波只做小幅修正），符号随机。
    Eigen::MatrixXd a(n, harmonics);
    Eigen::MatrixXd b(n, harmonics);
    for (unsigned int j = 0; j < n; ++j) {
      for (unsigned int l = 0; l < harmonics; ++l) {
        const double decay = 1.0 / static_cast<double>(l + 1);
        a(j, l) = unit(generator) * decay;
        b(j, l) = unit(generator) * decay;
      }
    }

    FourierTrajectory candidate;
    ObservabilityReport report;
    double scale = 0.0;
    if (!evaluate_candidate(a, b, candidate, report, scale)) {
      continue;
    }

    // 局部搜索：随机扰动一个系数，变好就接受。
    for (unsigned int iteration = 0; iteration < options.refine_iterations; ++iteration) {
      if (evaluations >= options.max_evaluations) {
        break;
      }
      Eigen::MatrixXd trial_a = a;
      Eigen::MatrixXd trial_b = b;
      const unsigned int j = joint_pick(generator);
      const unsigned int l = harmonic_pick(generator);
      const double delta = 0.2 * unit(generator);
      if (component_pick(generator) == 0) {
        trial_a(j, l) += delta;
      } else {
        trial_b(j, l) += delta;
      }

      FourierTrajectory trial;
      ObservabilityReport trial_report;
      double trial_scale = 0.0;
      if (!evaluate_candidate(trial_a, trial_b, trial, trial_report, trial_scale)) {
        continue;
      }
      if (is_better(trial_report.rank, trial_report.min_singular_value, report.rank,
        report.min_singular_value))
      {
        a = trial_a;
        b = trial_b;
        candidate = trial;
        report = trial_report;
        scale = trial_scale;
      }
    }

    if (!has_best || is_better(report.rank, report.min_singular_value, best_report.rank,
      best_report.min_singular_value))
    {
      has_best = true;
      best_trajectory = candidate;
      best_report = report;
      best_scale = scale;
    }
  }

  if (!has_best) {
    result.message = "优化失败：没有找到可行且可观测的激励轨迹";
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
    return result;
  }

  // 用与约束缩放相同的采样密度复算一次，作为最终对外报告的数字。
  result.trajectory = best_trajectory;
  result.observability = evaluateObservability(builder, best_trajectory, limit_samples,
    options.rank_tolerance);
  result.validation = validateFourierTrajectory(best_trajectory, limits, limit_samples);
  result.scale = best_scale;
  result.evaluations = evaluations;
  result.success = result.observability.success && result.validation.valid;
  if (!result.success) {
    result.message = "优化结果未能通过最终校验";
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
  }
  return result;
}

}  // namespace kdl_identification
