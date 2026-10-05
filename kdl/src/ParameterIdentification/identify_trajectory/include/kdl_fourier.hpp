// Copyright (c) 2026, kdl_identification authors.
// 教学用途：参数辨识用的傅里叶级数激励轨迹。
//
// ===========================================================================
// 一、为什么用傅里叶级数
// ===========================================================================
// 辨识的输入是 (q, q̇, q̈, τ)。要让最小二乘解稳定，激励必须"足够丰富"：
// 每个关节都要动、要覆盖较大的位形范围、速度和加速度要连续且量级够。
// 傅里叶级数（Swevers 等人的经典做法）几乎是为这件事量身定做的：
//
//     q_i(t) = q0_i + Σ_{l=1}^{H} [ a_il/(l·ω)·sin(l·ω·t) − b_il/(l·ω)·cos(l·ω·t) ]
//
//   · **周期性**：q(0) = q(T)，不需要额外处理"起步/停住"边界条件，
//     实际辨识时可以让机械臂连续跑几圈，多采几遍数据取平均。
//   · **解析导数**：q̇、q̈ 有闭式，不必对编码器信号差分（少一层噪声放大）：
//         q̇_i(t) = Σ [ a_il·cos(lωt) + b_il·sin(lωt) ]
//         q̈_i(t) = Σ l·ω·[ −a_il·sin(lωt) + b_il·cos(lωt) ]
//   · **参数少**：一条轨迹由 (q0, a, b, ω) 完全确定，便于做数值优化。
//
// ===========================================================================
// 二、怎么判断一条轨迹"好不好"
// ===========================================================================
// 用**可观测性**：把这条轨迹采成回归矩阵 W（见 kdl_regressor.hpp），
// 先按列归一化（消除 kg 与 kg·m² 的量纲差），再做 SVD。定义
//
//     rank      = 大于阈值的奇异值个数（能辨识的基参数个数）
//     σ_min⁺    = 最小的那个**非零**奇异值（越大越好，代表最弱方向的信号强度）
//     cond      = σ_max / σ_min⁺（越小越好）
//
// 激励优化的目标就是：先让 rank 尽量大，再让 σ_min⁺ 尽量大。
//
// ===========================================================================
// 三、约束怎么处理（"缩放到贴住约束"）
// ===========================================================================
// 位置/速度/加速度约束对幅值 (a, b) 是**齐次**的（同乘一个因子，三个量的
// 峰值同比放大）。于是优化分两步：
//   1) 先随机生成一个"形状"（各 (a, b) 的相对大小），中心取关节行程中点；
//   2) 找一个最大的缩放系数 λ，使 λ·形状刚好不越界（采样求峰值）。
// 这样每次候选都天然可行，搜索只需要比较可观测性。再叠加若干随机重启与
// 一次简单的坐标扰动局部搜索即可。

#ifndef KDL_PARAMETER_IDENTIFICATION__KDL_FOURIER_HPP_
#define KDL_PARAMETER_IDENTIFICATION__KDL_FOURIER_HPP_

#include <string>
#include <vector>

#include <Eigen/Core>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include "kdl_idynamics.hpp"
#include "kdl_regressor.hpp"

namespace kdl_identification
{

// ---------------------------------------------------------------------------
// 一、傅里叶轨迹
// ---------------------------------------------------------------------------

/**
 * @brief 一条周期性傅里叶激励轨迹。
 *
 * @note 公式见文件头。内部存的是**幅值系数** a、b（不是 q0、更不是级数项），
 *       因为约束缩放与优化都是在系数上做的。
 * @note 系数矩阵按"行 = 关节、列 = 谐波"存放：a(j, l) 对应第 j 个关节的
 *       第 l+1 次谐波正弦项（系数里已含 1/(l·ω)，见 sample()）。
 */
class FourierTrajectory
{
public:
  FourierTrajectory() = default;

  /**
   * @param harmonics   [in] 谐波数 H（≥ 1）。
   * @param period      [in] 周期 T [s]（> 0），基频 ω = 2π/T。
   * @param offset      [in] q0，各关节的偏置（一般取行程中点）。
   * @param sine_amplitudes   [in] a，n × H 矩阵（可为空表示全 0）。
   * @param cosine_amplitudes [in] b，n × H 矩阵（可为空表示全 0）。
   */
  FourierTrajectory(
    unsigned int harmonics, double period, const KDL::JntArray & offset,
    const Eigen::MatrixXd & sine_amplitudes, const Eigen::MatrixXd & cosine_amplitudes);

  /// @return 内部数据自洽时返回 true。
  bool valid() const;

  /// @return 关节数 n。
  unsigned int joints() const { return joints_; }

  /// @return 谐波数 H。
  unsigned int harmonics() const { return harmonics_; }

  /// @return 周期 T [s]。
  double period() const { return period_; }

  /// @return 基频 ω = 2π/T [rad/s]。
  double baseFrequency() const { return period_ > 0.0 ? 2.0 * kPi / period_ : 0.0; }

  /// @return 偏置 q0。
  const KDL::JntArray & offset() const { return offset_; }

  /// @return 正弦项幅值 a（n × H）。系数公式见 sample()。
  const Eigen::MatrixXd & sineAmplitudes() const { return sine_amplitudes_; }

  /// @return 余弦项幅值 b（n × H）。
  const Eigen::MatrixXd & cosineAmplitudes() const { return cosine_amplitudes_; }

  /**
   * @brief 在时刻 t 采样位置、速度、加速度。
   * @param t     [in]  时间 [s]（轨迹周期为 T，任意实数都有定义）。
   * @param q     [out] 关节角 [rad]；函数内部自动 resize。
   * @param qdot  [out] 关节速度 [rad/s]。
   * @param qddot [out] 关节加速度 [rad/s²]。
   * @return true 表示成功；false 表示轨迹无效。
   */
  bool sample(double t, KDL::JntArray & q, KDL::JntArray & qdot, KDL::JntArray & qddot) const;

  /**
   * @brief 在一个周期内等间隔采样，直接得到回归矩阵的输入。
   * @param samples [in]  采样点数（≥ 1），采样时刻为 t = k·T/samples（k = 0..samples−1）。
   * @param out     [out] 采样点序列。
   * @return true 表示成功。
   *
   * @note 取 k = 0..samples−1（不含右端点），避免周期边界处重复采一个点。
   */
  bool samplePeriod(unsigned int samples, std::vector<RegressorSample> & out) const;

  /**
   * @brief 按比例缩放幅值（a、b 同乘 factor），偏置不变。
   * @param factor [in] 缩放系数（≥ 0）。
   * @return 缩放后的新轨迹。
   *
   * @note 位置、速度、加速度的峰值都随 factor 线性放大，这正是
   *       "缩放到刚好贴住约束"（scaleToFitLimits）的基础。
   */
  FourierTrajectory scaled(double factor) const;

private:
  static constexpr double kPi = 3.14159265358979323846;

  bool is_valid_ = false;              ///< 数据是否自洽
  unsigned int joints_ = 0;            ///< 关节数
  unsigned int harmonics_ = 0;         ///< 谐波数
  double period_ = 0.0;                ///< 周期 [s]
  KDL::JntArray offset_;               ///< 偏置 q0（长度 n）
  Eigen::MatrixXd sine_amplitudes_;    ///< a（n × H）
  Eigen::MatrixXd cosine_amplitudes_;  ///< b（n × H）
};

// ---------------------------------------------------------------------------
// 二、约束与校验
// ---------------------------------------------------------------------------

/**
 * @brief 激励轨迹要满足的关节约束。
 *
 * @note 任何一个数组长度为 0 就表示"不校验这一项"。位置约束是
 *       q_min ≤ q(t) ≤ q_max；速度/加速度是 |q̇| ≤ max、|q̈| ≤ max。
 */
struct FourierLimits
{
  KDL::JntArray q_min;      ///< 关节下限 [rad]，长度 0 = 不校验
  KDL::JntArray q_max;      ///< 关节上限 [rad]，长度 0 = 不校验
  KDL::JntArray qdot_max;   ///< |q̇| 上限 [rad/s]，长度 0 = 不校验
  KDL::JntArray qddot_max;  ///< |q̈| 上限 [rad/s²]，长度 0 = 不校验

  /// @return 是否设置了位置约束（上下限长度都等于关节数）。
  bool checkPosition(unsigned int joints) const
  {
    return q_min.rows() == joints && q_max.rows() == joints;
  }

  /// @return 是否设置了速度约束。
  bool checkVelocity(unsigned int joints) const { return qdot_max.rows() == joints; }

  /// @return 是否设置了加速度约束。
  bool checkAcceleration(unsigned int joints) const { return qddot_max.rows() == joints; }
};

/**
 * @brief 轨迹约束校验结果。
 *
 * @note ratio_* 是"峰值 / 上限"的比值，刚好的量纲是 1：≤ 1 可行，> 1 越界。
 *       三个比值里最大的那个决定可行性，worst_joint 指出是哪个关节。
 */
struct FourierValidation
{
  bool valid = false;                 ///< 是否全部满足（或无需校验）
  double position_ratio = 0.0;        ///< max |q − 中点| / 半行程
  double velocity_ratio = 0.0;        ///< max |q̇| / 上限
  double acceleration_ratio = 0.0;    ///< max |q̈| / 上限
  unsigned int worst_joint = 0;       ///< 比值最大的关节下标
  std::string message;                ///< 可读说明（越界时指出关节与比值）
};

/**
 * @brief 采样校验一条轨迹是否满足约束。
 * @param trajectory        [in] 待校验轨迹。
 * @param limits            [in] 约束；未设置的项自动跳过。
 * @param samples_per_period [in] 每个周期的采样点数（越大越不容易漏掉峰值）。
 * @return FourierValidation。
 *
 * @note 位置约束用的是"相对行程中点"的等价形式 |q − 中点| ≤ 半行程。
 * @note 这是**数值**校验（按采样点求峰值），不是解析求极值；峰值的解析求解
 *       要对 H 个谐波叠加求极值，对教学场景不值得，采样足够密即可。
 */
FourierValidation validateFourierTrajectory(
  const FourierTrajectory & trajectory, const FourierLimits & limits,
  unsigned int samples_per_period = 200);

/**
 * @brief 把轨迹幅值缩放到"刚好贴住约束"。
 * @param trajectory         [in] 形状已定的轨迹（幅值任意）。
 * @param limits             [in] 约束。
 * @param samples_per_period [in] 峰值采样密度。
 * @return pair：第一个是缩放系数 λ（成功时 > 0），第二个是校验结果。
 *
 * @note 因为三个峰值都随幅值线性放大，λ = 1 / max(各项 ratio) 就是最大可行
 *       缩放；缩放后再校验一次，报告的是缩放后结果。
 * @note 若约束一个都没设置，返回 λ = 1（不缩放），这不常用于辨识。
 */
struct FourierScaling
{
  bool success = false;          ///< 是否成功（约束设置合法）
  double scale = 1.0;            ///< 缩放系数 λ
  FourierValidation validation;  ///< 缩放后的校验结果
  std::string message;           ///< 失败原因
};

FourierScaling scaleToFitLimits(
  const FourierTrajectory & trajectory, const FourierLimits & limits,
  unsigned int samples_per_period = 200);

// ---------------------------------------------------------------------------
// 三、可观测性
// ---------------------------------------------------------------------------

/**
 * @brief 一条轨迹的可观测性指标（由列归一化回归矩阵的奇异值给出）。
 */
struct ObservabilityReport
{
  bool success = false;                ///< 是否成功算出
  unsigned int rank = 0;               ///< 数值秩 = 可辨识基参数个数
  unsigned int total_parameters = 0;   ///< 参数总数 = 10 × 段数
  double min_singular_value = 0.0;     ///< 最小非零奇异值 σ_min⁺（越大越好）
  double condition_number = 0.0;       ///< σ_max / σ_min⁺（越小越好）
  std::string message;                 ///< 失败原因
};

/**
 * @brief 计算一条轨迹的可观测性指标。
 * @param builder   [in] 预先构造好的回归矩阵构造器（复用可避免重复建链）。
 * @param trajectory [in] 待评估轨迹。
 * @param samples_per_period [in] 每周期采样点数。
 * @param rank_tolerance [in] 判秩的相对阈值。
 * @return ObservabilityReport。
 */
ObservabilityReport evaluateObservability(
  const RegressorBuilder & builder, const FourierTrajectory & trajectory,
  unsigned int samples_per_period, double rank_tolerance = 1e-9);

// ---------------------------------------------------------------------------
// 四、激励轨迹优化
// ---------------------------------------------------------------------------

/**
 * @brief 激励轨迹优化的选项。
 */
struct ExcitationOptions
{
  unsigned int harmonics = 5;              ///< 谐波数 H
  double period = 4.0;                     ///< 周期 T [s]
  unsigned int samples_per_period = 50;    ///< 评估用采样密度
  unsigned int restarts = 15;              ///< 随机重启次数
  unsigned int refine_iterations = 8;      ///< 每次重启的局部搜索步数
  unsigned int max_evaluations = 150;      ///< 目标函数评估次数上限（控制耗时）
  unsigned int random_seed = 20261005u;    ///< 随机种子（保证可复现）
  double rank_tolerance = 1e-9;            ///< 判秩相对阈值
  KDL::Vector gravity = kdl_dynamics::defaultGravity();  ///< 重力向量
};

/**
 * @brief 激励轨迹优化的结果。
 */
struct ExcitationResult
{
  FourierTrajectory trajectory;      ///< 最优轨迹（已缩放到满足约束）
  ObservabilityReport observability; ///< 最优轨迹的可观测性（用较高采样密度复算）
  FourierValidation validation;      ///< 最优轨迹的约束校验
  double scale = 0.0;                ///< 相对"单位形状"的缩放系数
  unsigned int evaluations = 0;      ///< 实际目标函数评估次数
  bool success = false;              ///< 是否成功
  std::string message;               ///< 失败原因
};

/**
 * @brief 优化一条傅里叶激励轨迹，使其可观测性尽量好。
 * @param chain   [in] 运动学链（决定回归矩阵的结构）。
 * @param limits  [in] 关节约束；**位置约束必须提供**（用来确定激励中点与幅值）。
 * @param options [in] 选项。
 * @return ExcitationResult。
 *
 * @note 算法：随机重启生成形状 → 缩放到贴住约束 → 用"先比 rank、再比 σ_min⁺"
 *       的准则择优；每次重启后再做少量坐标扰动局部搜索。目标是让回归矩阵
 *       尽量"满秩且良态"。
 * @note 位置约束缺失时直接失败；速度/加速度缺失则只按已有约束缩放（可能给出
 *       很大的速度），因此正式辨识请三项都给全。
 */
ExcitationResult optimizeFourierExcitation(
  const KDL::Chain & chain, const FourierLimits & limits,
  const ExcitationOptions & options = ExcitationOptions());

}  // namespace kdl_identification

#endif  // KDL_PARAMETER_IDENTIFICATION__KDL_FOURIER_HPP_
