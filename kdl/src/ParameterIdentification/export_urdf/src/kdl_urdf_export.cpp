// Copyright (c) 2026, kdl_identification authors.
// 教学用途：kdl_urdf_export.hpp 中声明的函数在这里落地。
//
// 文件顺序与头文件一一对应：
//   一、参数补全      —— 零空间投影（线性代数）
//   二、可表示性检查  —— 与 kdl_regressor 的 applyParameters 同一判据
//   三、导出          —— β → <inertial> 换算 + 文本外科替换
//   四、往返校验      —— 用 kdl_parser 重新解析，验证导出没写错

#include "kdl_urdf_export.hpp"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <rclcpp/logging.hpp>

#include "kdl_regressor.hpp"  // kParamsPerLink / extractParameters
#include "kdl_tools.hpp"      // buildChainFromUrdfFile

namespace kdl_identification
{
namespace
{

/// 所有日志统一前缀，方便在终端里一眼看出是谁打的。
constexpr const char * kLogTag = "kdl_identification";

/// 判定"质量为零"的阈值（kg）。与 kdl_regressor 的 kMassEpsilon 保持一致。
constexpr double kMassEpsilon = 1e-12;

/// 判定"第一阶矩为零"的阈值（kg·m）。与 kdl_regressor 的 kFirstMomentEpsilon 一致。
constexpr double kFirstMomentEpsilon = 1e-12;

/// 列范数的相对阈值，与 LeastSquaresOptions::column_tolerance 保持一致。
constexpr double kColumnTolerance = 1e-10;

/**
 * @brief 把 double 格式化成可写进 XML 的字符串。
 * @note 用 15 位有效数字：足够让"写出 → 解析回来"的往返误差落在 1e-12 以内，
 *       同时不会像 %.17g 那样把浮点噪声也印出来。
 */
std::string formatDouble(double value)
{
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.15g", value);
  return std::string(buffer);
}

/// @return 字符串 s 从 index 起是否全是空白。
bool isBlankFrom(const std::string & s, std::size_t index)
{
  for (std::size_t i = index; i < s.size(); ++i) {
    if (!std::isspace(static_cast<unsigned char>(s[i]))) {
      return false;
    }
  }
  return true;
}

/**
 * @brief 在一个 XML 标签文本里找属性值，要求属性名前面是"分隔符"。
 * @param tag   [in] 形如 `<link\n    name="link1">` 的标签文本。
 * @param attr  [in] 属性名，如 "name"。
 * @param value [in] 期望的属性值。
 * @return true 表示该标签确实有这个属性且值相等。
 *
 * @note 必须检查前缀分隔符：`filename="..."` 里也含有子串 `name="`，
 *       不检查就会把 mesh 的 filename 误当成 link 的 name。
 */
bool attributeEquals(const std::string & tag, const std::string & attr, const std::string & value)
{
  const std::string needle = attr + "=\"";
  std::size_t pos = 0;
  while (true) {
    pos = tag.find(needle, pos);
    if (pos == std::string::npos) {
      return false;
    }
    const bool boundary = (pos == 0) ||
      std::isspace(static_cast<unsigned char>(tag[pos - 1])) || tag[pos - 1] == '<';
    if (!boundary) {
      pos += 1;
      continue;
    }
    const std::size_t value_begin = pos + needle.size();
    const std::size_t value_end = tag.find('"', value_begin);
    if (value_end == std::string::npos) {
      return false;
    }
    if (tag.compare(value_begin, value_end - value_begin, value) == 0) {
      return true;
    }
    pos = value_end;
  }
}

/// @return 从 index 往前数的连续空白（即该标签所在行的缩进）。
std::string leadingIndent(const std::string & text, std::size_t index)
{
  std::size_t begin = index;
  while (begin > 0 && (text[begin - 1] == ' ' || text[begin - 1] == '\t')) {
    --begin;
  }
  return text.substr(begin, index - begin);
}

/**
 * @brief 定位 `<link name="X"> ... </link>` 在文本中的区间。
 * @param xml        [in]  URDF 全文。
 * @param link_name  [in]  目标 link 名。
 * @param open_end   [out] 开标签 `>` 之后的位置。
 * @param close_begin[out] `</link>` 的起始位置。
 * @param open_begin [out] `<link` 的起始位置。
 * @param message    [out] 失败原因。
 * @return true 表示找到。
 */
bool findLinkSpan(
  const std::string & xml, const std::string & link_name, std::size_t & open_begin,
  std::size_t & open_end, std::size_t & close_begin, std::string & message)
{
  std::size_t pos = 0;
  while (true) {
    const std::size_t found = xml.find("<link", pos);
    if (found == std::string::npos) {
      break;
    }
    // 必须是一个完整的标签名：后面跟空白或 '>'，避免匹配到 <link_xxx。
    const char after = (found + 5 < xml.size()) ? xml[found + 5] : '\0';
    if (!(std::isspace(static_cast<unsigned char>(after)) || after == '>')) {
      pos = found + 5;
      continue;
    }
    const std::size_t tag_end = xml.find('>', found);
    if (tag_end == std::string::npos) {
      break;
    }
    const std::string tag = xml.substr(found, tag_end - found + 1);
    if (attributeEquals(tag, "name", link_name)) {
      const std::size_t close = xml.find("</link>", tag_end);
      if (close == std::string::npos) {
        message = "URDF 里 link \"" + link_name + "\" 没有配对的 </link>";
        return false;
      }
      open_begin = found;
      open_end = tag_end + 1;
      close_begin = close;
      return true;
    }
    pos = tag_end + 1;
  }
  message = "URDF 里找不到 link \"" + link_name + "\"";
  return false;
}

/**
 * @brief 由 10 维参数块生成 `<inertial>` 文本。
 * @param block     [in] m, h, I_ref（相对 link 坐标系原点）。
 * @param link_name [in] 段名（仅用于报错）。
 * @param indent    [in] `<inertial>` 前面的缩进。
 * @param text      [out] 生成的 XML 片段（不含前导缩进）。
 * @param info      [out] 供调用者打印的换算结果。
 * @param message   [out] 失败原因。
 * @return true 表示成功。
 *
 * @note 换算就是平行轴定理的逆运算：
 *           I_cog = I_ref − m·(|c|²·Id − c·cᵀ)，  c = h / m
 *       然后写 origin.xyz = c、inertia = I_cog。KDL 解析时会再把它加回去，
 *       于是往返无损（见 verifyExportedUrdf）。
 * @note 一律写 rpy="0 0 0"：把质心放在 xyz、姿态与 link 系对齐，既是最通用的
 *       写法，也绕开了"URDF 惯量的 rpy 旋转"这一层容易出错的换算。
 */
bool buildInertialBlock(
  const Eigen::VectorXd & block, const std::string & link_name, const std::string & indent,
  std::string & text, UrdfLinkExport & info, std::string & message)
{
  const double m = block[0];
  const Eigen::Vector3d h(block[1], block[2], block[3]);

  Eigen::Matrix3d i_ref;
  i_ref << block[4], block[5], block[6], block[5], block[7], block[8], block[6], block[8], block[9];

  Eigen::Vector3d cog = Eigen::Vector3d::Zero();
  Eigen::Matrix3d i_cog = i_ref;
  if (m > kMassEpsilon) {
    cog = h / m;
    i_cog = i_ref - m * (cog.dot(cog) * Eigen::Matrix3d::Identity() - cog * cog.transpose());
  } else if (h.norm() > kFirstMomentEpsilon) {
    message = "段 \"" + link_name + "\" 的质量接近 0 但第一阶矩不为 0（" +
      formatDouble(h.norm()) + "），URDF 无法表示；请先用 completeWithPrior() 补全参数";
    return false;
  }

  info.link_name = link_name;
  info.mass = m;
  info.cog = cog;
  info.inertia_at_cog = i_cog;

  const std::string child_indent = indent + "  ";
  std::ostringstream out;
  out << "<inertial>\n"
      << child_indent << "<origin xyz=\"" << formatDouble(cog.x()) << " " << formatDouble(cog.y())
      << " " << formatDouble(cog.z()) << "\" rpy=\"0 0 0\" />\n"
      << child_indent << "<mass value=\"" << formatDouble(m) << "\" />\n"
      << child_indent << "<inertia ixx=\"" << formatDouble(i_cog(0, 0)) << "\" ixy=\""
      << formatDouble(i_cog(0, 1)) << "\" ixz=\"" << formatDouble(i_cog(0, 2)) << "\" iyy=\""
      << formatDouble(i_cog(1, 1)) << "\" iyz=\"" << formatDouble(i_cog(1, 2)) << "\" izz=\""
      << formatDouble(i_cog(2, 2)) << "\" />\n"
      << indent << "</inertial>";
  text = out.str();
  message.clear();
  return true;
}

/// 读整个文本文件。
bool readTextFile(const std::string & path, std::string & text, std::string & message)
{
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    message = "无法打开文件：" + path;
    return false;
  }
  std::ostringstream buffer;
  buffer << input.rdbuf();
  text = buffer.str();
  if (text.empty()) {
    message = "文件为空：" + path;
    return false;
  }
  message.clear();
  return true;
}

/**
 * @brief 把参数逐段投影到物理可行集（质量下限 + 质心惯量正定 + 三角不等式）。
 * @param parameters [in] 长度须为 10 的整数倍。
 * @return 投影后的参数（不会破坏段数结构）。
 *
 * @note 投影只在**每段内部**做（不动跨段的耦合），因为可行集本身是逐段解耦的
 *       笛卡尔积：每段的 (m, h, I_ref) 只需满足自己那一个 LMI。
 * @note 具体做法：算 I_cog，特征分解，把特征值夹到 ≥ floor（floor 取最大特征值
 *       的 1e-4 与 1e-9 的较大者），再把最大主惯量夹到不超过另外两者之和
 *       （三角不等式），最后用同一个平行轴项换回 I_ref。
 */
Eigen::VectorXd projectToPhysicalFeasibility(const Eigen::VectorXd & parameters)
{
  Eigen::VectorXd projected = parameters;
  if (parameters.size() == 0 || parameters.size() % kParamsPerLink != 0) {
    return projected;
  }
  const unsigned int segments = static_cast<unsigned int>(parameters.size() / kParamsPerLink);

  for (unsigned int s = 0; s < segments; ++s) {
    const Eigen::VectorXd block = parameters.segment<kParamsPerLink>(kParamsPerLink * s);
    double m = block[0];
    if (!(m > kMassEpsilon)) {
      m = kMassEpsilon;  // 质量必须为正；第一阶矩保持原样（这一步之后仍需重新检查）
    }
    const Eigen::Vector3d h(block[1], block[2], block[3]);
    const Eigen::Vector3d cog = h / m;
    Eigen::Matrix3d i_ref;
    i_ref << block[4], block[5], block[6], block[5], block[7], block[8], block[6], block[8], block[9];
    const Eigen::Matrix3d parallel =
      m * (cog.dot(cog) * Eigen::Matrix3d::Identity() - cog * cog.transpose());
    const Eigen::Matrix3d i_cog = i_ref - parallel;

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(
      0.5 * (i_cog + i_cog.transpose()));
    Eigen::Vector3d eigenvalues = solver.eigenvalues();  // 升序
    const Eigen::Matrix3d eigenvectors = solver.eigenvectors();

    const double floor = std::max(1e-9, 1e-4 * eigenvalues(2));
    eigenvalues = eigenvalues.cwiseMax(floor);
    if (eigenvalues(2) > eigenvalues(0) + eigenvalues(1)) {
      eigenvalues(2) = 0.999 * (eigenvalues(0) + eigenvalues(1));
    }

    const Eigen::Matrix3d i_cog_projected =
      eigenvectors * eigenvalues.asDiagonal() * eigenvectors.transpose();
    const Eigen::Matrix3d i_ref_projected = i_cog_projected + parallel;

    auto out = projected.segment<kParamsPerLink>(kParamsPerLink * s);
    out[0] = m;
    out[1] = h.x();
    out[2] = h.y();
    out[3] = h.z();
    out[4] = i_ref_projected(0, 0);
    out[5] = i_ref_projected(0, 1);
    out[6] = i_ref_projected(0, 2);
    out[7] = i_ref_projected(1, 1);
    out[8] = i_ref_projected(1, 2);
    out[9] = i_ref_projected(2, 2);
  }
  return projected;
}

}  // namespace

// ---------------------------------------------------------------------------
// 一、参数补全
// ---------------------------------------------------------------------------

PhysicalFeasibility checkPhysicalFeasibility(const Eigen::VectorXd & parameters)
{
  PhysicalFeasibility result;
  if (parameters.size() == 0 || parameters.size() % kParamsPerLink != 0) {
    result.message = "参数向量长度必须是 10 的整数倍，实际为 " + std::to_string(parameters.size());
    return result;
  }
  result.segments = static_cast<unsigned int>(parameters.size() / kParamsPerLink);
  result.min_mass = std::numeric_limits<double>::max();
  result.min_inertia_eigenvalue = std::numeric_limits<double>::max();

  for (unsigned int s = 0; s < result.segments; ++s) {
    const Eigen::VectorXd block = parameters.segment<kParamsPerLink>(kParamsPerLink * s);
    const double m = block[0];
    result.min_mass = std::min(result.min_mass, m);

    if (!(m > kMassEpsilon)) {
      result.message = "段 " + std::to_string(s) + " 的质量为 " + formatDouble(m) + "（须 > 0）";
      result.worst_segment = s;
      return result;
    }

    const Eigen::Vector3d h(block[1], block[2], block[3]);
    const Eigen::Vector3d cog = h / m;
    Eigen::Matrix3d i_ref;
    i_ref << block[4], block[5], block[6], block[5], block[7], block[8], block[6], block[8], block[9];
    const Eigen::Matrix3d i_cog =
      i_ref - m * (cog.dot(cog) * Eigen::Matrix3d::Identity() - cog * cog.transpose());

    // 质心惯量必须正定：对称化后取特征值（闭式解，3x3 很便宜）。
    const Eigen::Matrix3d symmetric = 0.5 * (i_cog + i_cog.transpose());
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(symmetric);
    const double min_eigenvalue = solver.eigenvalues().minCoeff();
    result.min_inertia_eigenvalue = std::min(result.min_inertia_eigenvalue, min_eigenvalue);
    if (!(min_eigenvalue > 0.0)) {
      result.message = "段 " + std::to_string(s) + " 的质心惯量非正定（最小特征值 " +
        formatDouble(min_eigenvalue) + "）";
      result.worst_segment = s;
      return result;
    }

    // 主惯量的三角不等式（刚体惯量的必要条件，MuJoCo 等也会检查）。
    const double ixx = symmetric(0, 0);
    const double iyy = symmetric(1, 1);
    const double izz = symmetric(2, 2);
    if (!(ixx + iyy >= izz && iyy + izz >= ixx && izz + ixx >= iyy)) {
      result.message = "段 " + std::to_string(s) + " 的质心惯量违反三角不等式（主惯量 " +
        formatDouble(ixx) + " / " + formatDouble(iyy) + " / " + formatDouble(izz) + "）";
      result.worst_segment = s;
      return result;
    }
  }

  result.feasible = true;
  result.message.clear();
  return result;
}

CompletionResult completeWithPrior(
  const Eigen::MatrixXd & regressor, const Eigen::VectorXd & identified,
  const Eigen::VectorXd & prior, const CompletionOptions & options)
{
  CompletionResult result;
  result.parameters = identified;

  if (regressor.rows() == 0 || regressor.cols() == 0) {
    result.message = "回归矩阵为空，无法做零空间补全";
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
    return result;
  }
  if (identified.size() != regressor.cols() || prior.size() != regressor.cols()) {
    result.message = "参数向量长度（" + std::to_string(identified.size()) + " / " +
      std::to_string(prior.size()) + "）必须等于回归矩阵列数 " +
      std::to_string(regressor.cols());
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
    return result;
  }

  const unsigned int cols = static_cast<unsigned int>(regressor.cols());

  // ---- 0) 度量尺度 s_j：默认取先验的量级，下限用"同类型参数的最大绝对值 ×1e-3" ----
  // 类型划分与 kParamsPerLink 的布局一致：0=m，1..3=m·c，4..9=转动惯量。
  Eigen::VectorXd scale(cols);
  if (options.scale.size() == static_cast<Eigen::Index>(cols)) {
    scale = options.scale;
  } else if (options.scale.size() != 0) {
    result.message = "options.scale 的长度（" + std::to_string(options.scale.size()) +
      "）必须等于回归矩阵列数 " + std::to_string(cols);
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
    return result;
  } else {
    for (unsigned int j = 0; j < cols; ++j) {
      const unsigned int type = j % kParamsPerLink;
      const unsigned int type_begin = (type == 0) ? 0u : ((type <= 3) ? 1u : 4u);
      const unsigned int type_end = (type == 0) ? 1u : ((type <= 3) ? 4u : 10u);
      double type_max = 0.0;
      for (unsigned int k = type_begin; k < type_end; ++k) {
        for (unsigned int segment = 0; segment * kParamsPerLink + k < cols; ++segment) {
          type_max = std::max(type_max, std::abs(prior(segment * kParamsPerLink + k)));
        }
      }
      scale(j) = std::max(std::abs(prior(j)), 1e-3 * type_max);
    }
  }
  for (unsigned int j = 0; j < cols; ++j) {
    if (!(scale(j) > 0.0)) {
      scale(j) = 1.0;  // 兜底：先验全零时退化成欧氏度量
    }
  }

  // ---- 1) 判秩 + 零空间基：对列归一化后的 W 做 SVD（与 kdl_lsq 同策） ----
  // 可辨识子空间 row(W) 与度量无关，是个固定子空间；但"从奇异值排序上把它切出来"
  // 这件事**依赖缩放**（kg 与 kg·m² 差几个数量级会打乱排序）。所以这里用
  // 列归一化后的矩阵定出秩与零空间基，之后所有度量都只作用在零空间内部。
  // 恒零列（任何运动都不影响 τ 的参数）用相对阈值识别，它们整体属于零空间。
  Eigen::VectorXd column_norm(cols);
  for (unsigned int j = 0; j < cols; ++j) {
    column_norm(j) = regressor.col(j).norm();
  }
  const double max_norm = column_norm.maxCoeff();
  Eigen::MatrixXd normalized = regressor;
  for (unsigned int j = 0; j < cols; ++j) {
    if (max_norm > 0.0 && column_norm(j) > max_norm * kColumnTolerance) {
      normalized.col(j) /= column_norm(j);
    } else {
      normalized.col(j).setZero();
      column_norm(j) = 1.0;  // 该列本来就恒零，取 1 便于下面反缩放
    }
  }

  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(
    normalized, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd singular_values = svd.singularValues();
  if (singular_values.size() == 0 || singular_values(0) <= 0.0) {
    result.message = "回归矩阵全零，秩为 0，无法做零空间补全";
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
    return result;
  }
  unsigned int rank = 0;
  for (Eigen::Index i = 0; i < singular_values.size(); ++i) {
    if (singular_values(i) > options.rank_tolerance * singular_values(0)) {
      ++rank;
    }
  }
  if (rank == 0) {
    result.message = "回归矩阵数值秩为 0，无法做零空间补全";
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
    return result;
  }
  result.rank = rank;
  const unsigned int null_dim = cols - rank;
  result.null_space_dimension = null_dim;

  // W̃ = W·D⁻¹（D = diag(列范数)）⇒ null(W) = D⁻¹·null(W̃)。
  // 取 W̃ 的后 null_dim 个右奇异向量（它们张成 null(W̃)），再按列范数反缩放。
  Eigen::MatrixXd null_basis(cols, null_dim);
  for (unsigned int i = 0; i < null_dim; ++i) {
    const Eigen::Index column = static_cast<Eigen::Index>(rank + i);
    if (column >= svd.matrixV().cols()) {
      result.message = "零空间基提取失败：右奇异向量维数不足";
      RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
      return result;
    }
    null_basis.col(i) = svd.matrixV().col(column);
    for (unsigned int j = 0; j < cols; ++j) {
      null_basis(j, i) /= column_norm(j);
    }
  }

  // ---- 2) 在零空间内按 S 度量求解 ----
  //     min ‖S·(β_id + N·z − β_prior)‖  ⇒  z = (NᵀS²N)⁻¹·NᵀS²·(β_prior − β_id)。
  // 零空间维数只有十几，直接解正规方程即可；用列主元 QR 保证数值稳定。
  const Eigen::VectorXd gap = prior - identified;
  Eigen::MatrixXd weighted_null = null_basis;
  for (unsigned int j = 0; j < cols; ++j) {
    weighted_null.row(j) *= scale(j) * scale(j);  // S²·N
  }
  const Eigen::MatrixXd lhs = null_basis.transpose() * weighted_null;  // NᵀS²N
  const Eigen::VectorXd rhs = weighted_null.transpose() * gap;         // NᵀS²g
  const Eigen::VectorXd z = lhs.colPivHouseholderQr().solve(rhs);

  result.parameters = identified + null_basis * z;
  result.correction_norm = (result.parameters - prior).norm();
  result.prior_gap_norm = gap.norm();

  // ---- 3) 可选的物理可行性投影（交替投影 / POCS） ----
  // 补全只保证 W·β = W·β_id，**不保证** β 物理可行：可行集是一个 LMI
  // （伪惯量矩阵 J = [[½tr(I_ref)Id − I_ref, h], [hᵀ, m]] ≻ 0，是 LMI；
  //  注意不能写成 m·I_ref − |h|²Id + hhᵀ ≻ 0 —— 那是 BMI、非凸），
  // 与"τ 等价的仿射集"并不重合。
  // 两个集合都是凸的，于是在两者之间交替投影（POCS）：
  //     ① 投到可行集：逐段把 I_cog 的特征值夹到正、并满足三角不等式；
  //     ② 投回仿射集：在 S 度量下取仿射集里离当前点最近的点。
  // 若两集相交，迭代收敛到交集中的一点（此时**既**拟合 τ **又**物理可行）；
  // 若不相交（噪声把仿射集整体推离可行集时会发生），就取迭代过程中"可行且
  // 力矩偏差最小"的那个点，并把代价如实记在 torque_residual_norm 里。
  if (options.enforce_physical_feasibility) {
    const PhysicalFeasibility at_one = checkPhysicalFeasibility(result.parameters);
    result.physically_feasible = at_one.feasible;

    if (!at_one.feasible) {
      const PhysicalFeasibility prior_feasibility = checkPhysicalFeasibility(prior);
      if (!prior_feasibility.feasible) {
        result.message = "先验参数本身就不物理可行（" + prior_feasibility.message +
          "），无法投影";
        RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
        return result;
      }

      // 仿射集上的 S 度量投影：z = (NᵀS²N)⁻¹·NᵀS²·(target − β_id)。
      // 注意返回类型必须显式写成 Eigen::VectorXd：若交给 auto 推导，lambda 会把
      // Eigen 的**表达式模板**返回出去，而它引用的临时量已经析构（未定义行为，
      // 实测直接 std::bad_alloc）。
      const Eigen::MatrixXd normal_matrix = lhs;  // NᵀS²N
      const auto project_onto_affine =
        [&](const Eigen::VectorXd & target) -> Eigen::VectorXd {
          const Eigen::VectorXd rhs_target = weighted_null.transpose() * (target - identified);
          const Eigen::VectorXd z_target = normal_matrix.colPivHouseholderQr().solve(rhs_target);
          return identified + null_basis * z_target;
        };

      Eigen::VectorXd current = result.parameters;
      Eigen::VectorXd best = current;
      double best_torque_error = std::numeric_limits<double>::max();
      bool found_feasible = false;

      for (unsigned int iteration = 0; iteration < options.feasibility_iterations; ++iteration) {
        current = project_onto_affine(projectToPhysicalFeasibility(current));
        if (checkPhysicalFeasibility(current).feasible) {
          const double torque_error = (regressor * current - regressor * identified).norm();
          if (torque_error < best_torque_error) {
            best_torque_error = torque_error;
            best = current;
            found_feasible = true;
          }
        }
      }

      if (found_feasible) {
        result.parameters = best;
        result.physically_feasible = true;
        result.feasibility_fraction = 1.0;  // 交集中的点：力矩拟合与可行性同时满足
      } else {
        // 两集不相交：退回"先验 → 补全结果"线段上可行的最大比例 t。
        double low = 0.0;
        double high = 1.0;
        for (unsigned int iteration = 0; iteration < 60; ++iteration) {
          const double middle = 0.5 * (low + high);
          if (checkPhysicalFeasibility(prior + middle * (result.parameters - prior)).feasible) {
            low = middle;
          } else {
            high = middle;
          }
        }
        result.parameters = prior + low * (result.parameters - prior);
        result.physically_feasible = checkPhysicalFeasibility(result.parameters).feasible;
        result.feasibility_fraction = low;
      }
    }
  }

  // 注意：correction_norm 必须在这里**重算**。上面那次赋值发生在可行性退让之前，
  // 退让后 parameters 已经变成别的点；不重算就会报出一个与实际返回值不符的数字
  // （实测退让到 t=0 时仍报 265，而实际返回的就是先验、距离应为 0）。
  result.correction_norm = (result.parameters - prior).norm();
  result.torque_residual_norm = (regressor * result.parameters - regressor * identified).norm();

  if (!result.parameters.allFinite()) {
    result.message = "零空间补全得到非有限值（回归矩阵病态）";
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
    return result;
  }

  result.success = true;
  result.message.clear();
  return result;
}

// ---------------------------------------------------------------------------
// 二、可表示性检查
// ---------------------------------------------------------------------------

bool parametersRepresentableInUrdf(
  const KDL::Chain & chain, const Eigen::VectorXd & parameters, std::string & message)
{
  const unsigned int segments = chain.getNrOfSegments();
  if (parameters.size() != static_cast<Eigen::Index>(kParamsPerLink * segments)) {
    message = "参数向量长度必须等于 10 × 段数（" + std::to_string(kParamsPerLink * segments) +
      "），实际为 " + std::to_string(parameters.size());
    return false;
  }

  for (unsigned int s = 0; s < segments; ++s) {
    const Eigen::VectorXd block = parameters.segment<kParamsPerLink>(kParamsPerLink * s);
    const double m = block[0];
    const Eigen::Vector3d h(block[1], block[2], block[3]);
    if (!(m > kMassEpsilon) && h.norm() > kFirstMomentEpsilon) {
      message = "段 \"" + chain.getSegment(s).getName() + "\" 的质量为 " + formatDouble(m) +
        "（≈0）但第一阶矩为 " + formatDouble(h.norm()) + "（≠0），URDF 无法表示";
      return false;
    }
  }

  message.clear();
  return true;
}

// ---------------------------------------------------------------------------
// 三、导出
// ---------------------------------------------------------------------------

UrdfExportResult exportUrdfWithParameters(
  const std::string & input_urdf, const KDL::Chain & chain, const Eigen::VectorXd & parameters,
  const std::string & output_urdf)
{
  UrdfExportResult result;
  result.segments = chain.getNrOfSegments();
  result.output_file = output_urdf;

  if (!parametersRepresentableInUrdf(chain, parameters, result.message)) {
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "导出 URDF 失败：%s", result.message.c_str());
    return result;
  }

  std::string xml;
  if (!readTextFile(input_urdf, xml, result.message)) {
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "导出 URDF 失败：%s", result.message.c_str());
    return result;
  }

  // 逐段改写。每改一次文本长度都会变，所以每次重新定位区间（文件很小，代价可忽略）。
  for (unsigned int s = 0; s < result.segments; ++s) {
    const std::string link_name = chain.getSegment(s).getName();
    const Eigen::VectorXd block = parameters.segment<kParamsPerLink>(kParamsPerLink * s);

    std::size_t open_begin = 0;
    std::size_t open_end = 0;
    std::size_t close_begin = 0;
    if (!findLinkSpan(xml, link_name, open_begin, open_end, close_begin, result.message)) {
      RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "导出 URDF 失败：%s", result.message.c_str());
      return result;
    }

    // 已有 <inertial> 就沿用它的缩进；没有就用 link 的子级默认缩进（4 空格）。
    const std::size_t inertial_begin = xml.find("<inertial", open_end);
    const bool has_inertial = (inertial_begin != std::string::npos && inertial_begin < close_begin);
    const std::string indent = has_inertial ? leadingIndent(xml, inertial_begin) : "    ";

    UrdfLinkExport info;
    std::string block_text;
    if (!buildInertialBlock(block, link_name, indent, block_text, info, result.message)) {
      RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "导出 URDF 失败：%s", result.message.c_str());
      return result;
    }
    info.replaced_existing = has_inertial;

    if (has_inertial) {
      const std::size_t tag_end = xml.find('>', inertial_begin);
      if (tag_end == std::string::npos || tag_end >= close_begin) {
        result.message = "link \"" + link_name + "\" 的 <inertial> 标签不完整";
        RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "导出 URDF 失败：%s", result.message.c_str());
        return result;
      }
      std::size_t inertial_end = 0;
      const std::size_t last_char = xml.find_last_not_of(" \t\r\n", tag_end - 1);
      if (last_char != std::string::npos && xml[last_char] == '/') {
        inertial_end = tag_end + 1;  // 自闭合 <inertial ... />
      } else {
        const std::size_t close = xml.find("</inertial>", tag_end);
        if (close == std::string::npos || close >= close_begin) {
          result.message = "link \"" + link_name + "\" 的 <inertial> 没有配对的 </inertial>";
          RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "导出 URDF 失败：%s", result.message.c_str());
          return result;
        }
        inertial_end = close + std::strlen("</inertial>");
      }
      xml.replace(inertial_begin, inertial_end - inertial_begin, block_text);
    } else {
      // 没有 <inertial>：插在 </link> 前面，并补一个换行 + 缩进。
      xml.insert(close_begin, block_text + "\n" + indent);
    }

    result.links.push_back(info);
  }

  // 写文件（父目录可能不存在）。
  const std::filesystem::path path(output_urdf);
  const std::filesystem::path parent = path.parent_path();
  if (!parent.empty()) {
    std::error_code error;
    std::filesystem::create_directories(parent, error);
    if (error) {
      result.message = "无法创建输出目录 " + parent.string() + "：" + error.message();
      RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
      return result;
    }
  }
  std::ofstream output(output_urdf, std::ios::binary);
  if (!output) {
    result.message = "无法写入文件：" + output_urdf;
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
    return result;
  }
  output << xml;
  output.close();
  if (!output) {
    result.message = "写入文件失败（磁盘/权限）：" + output_urdf;
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
    return result;
  }

  result.success = true;
  result.message.clear();
  return result;
}

// ---------------------------------------------------------------------------
// 四、往返校验
// ---------------------------------------------------------------------------

UrdfRoundTripCheck verifyExportedUrdf(
  const std::string & urdf_file, const Eigen::VectorXd & expected_parameters)
{
  UrdfRoundTripCheck result;

  KDL::Chain chain;
  if (!kdl_tools::buildChainFromUrdfFile(urdf_file, chain)) {
    result.message = "无法从导出的 URDF 建链：" + urdf_file;
    return result;
  }
  result.segments = chain.getNrOfSegments();

  Eigen::VectorXd actual;
  std::string message;
  if (!extractParameters(chain, actual, message)) {
    result.message = "从导出的 URDF 读取参数失败：" + message;
    return result;
  }
  if (actual.size() != expected_parameters.size()) {
    result.message = "导出的 URDF 参数个数 " + std::to_string(actual.size()) + " 与期望 " +
      std::to_string(expected_parameters.size()) + " 不一致（结构被改动？）";
    return result;
  }

  const Eigen::VectorXd difference = (actual - expected_parameters).cwiseAbs();
  result.max_parameter_error = difference.size() > 0 ? difference.maxCoeff() : 0.0;
  for (Eigen::Index i = 0; i < difference.size(); ++i) {
    const double denominator = std::max(1.0, std::abs(expected_parameters(i)));
    result.max_relative_error = std::max(result.max_relative_error, difference(i) / denominator);
  }

  result.success = true;
  result.message.clear();
  return result;
}

}  // namespace kdl_identification
