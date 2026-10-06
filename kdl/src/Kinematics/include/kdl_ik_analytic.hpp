// Copyright (c) 2026, kdl_kinematics authors.
// 教学用途：4 轴臂（平面 2R + 2 自由度腕部）的**解析逆解**。
//
// ===========================================================================
// 一、为什么需要单独一份解析解
// ===========================================================================
// kdl_ik.hpp 里的 solveIkLma / solveIkNrJl 是**通用数值解**：它们把末端位姿当成
// 6 维任务空间去逼近，对任何 KDL::Chain 都能跑。但 4 轴臂只有 4 个自由度，
// 拿 6 维位姿去要求它是**超定**的 —— 除非目标恰好落在可达流形上，否则无解。
//
// 工业上 4 轴臂（SCARA、码垛机器人等）一律用解析解，因为这类构型几乎总能解耦。
// 本文件就是针对下面这类构型写的闭式解：
//
//   joint1 ── joint2 ── joint3 ── joint4
//    └──── 平行轴，构成平面 2R ────┘  └─ 2 自由度腕部 ─┘
//
// 判据（extractPlanar4ArmGeometry 会逐条校验，不满足就返回失败）：
//   1) joint1 与 joint2 的轴平行；
//   2) joint3 的轴垂直于 joint1 的轴（即 joint3 的轴始终落在臂平面内）；
//   3) joint4 的轴与 joint3 的轴相交（存在"腕心"），且 joint4 的轴与 joint3 的
//      轴正交。
//
// ===========================================================================
// 二、任务空间：4 维，不是 6 维
// ===========================================================================
// 本文件的接口只吃**工具点位置**（3 维）+ **绕工具轴的自转角**（1 维），共 4 维，
// 正好等于自由度数。工具轴的**方向**不需要给 —— 它由位置唯一决定（给定肘部分支）。
//
//   输入：target_position（工具点在基座系的位置）
//         target_roll（绕工具轴的自转角，见第五节）
//         q_seed（用于选分支、也用于热启动牛顿迭代）
//   输出：q（4 个关节角）
//
// ===========================================================================
// 三、几何量（全部从 KDL::Chain 提取，不硬编码）
// ===========================================================================
// 记 n 为 joint1/joint2 的公共轴方向，(u, v) 为臂平面内的一组正交基（u × v = n）。
// 把任意点 X 的坐标分解成"平面内 (X·u, X·v)"与"平面外 X·n"两部分。
//
//   量          含义
//   ────────────────────────────────────────────────────────────────
//   origin      joint1 轴上的一点
//   axis        n
//   L1          joint1 轴 → joint2 轴的**垂距**（平面内）
//   L2          joint2 轴 → joint3 轴的**垂距**（平面内）
//   out_of_plane joint3 原点的平面外坐标（常量）
//   delta1      q1 与"平面内第一段连杆方位角"之间的结构偏置
//   delta2      q2 与"平面内第二段连杆相对转角"之间的结构偏置
//   wrist_h     joint3 原点沿 joint3 轴到"腕心"的距离
//   tool_local  joint3 原点 → 工具点，**在 link3 坐标系里**（常量向量）
//
// 平面 2R 的角度关系（实测精确成立）：
//
//   θ1     = q1 + delta1                          （第一段连杆的方位角）
//   θ2_abs = θ1 − q2 + delta2                     （第二段连杆的方位角，绝对值）
//
// ===========================================================================
// 四、求解步骤
// ===========================================================================
// 记 c = tool_local（joint3 原点 → 工具点，在 link3 系里固定），
// 把它按 joint3 轴分解：c = h·ẑ + c⊥（ẑ 是 link3 系里的 (0,0,1)，即 joint3 轴），
// 于是 h = wrist_h、|c⊥| = |c|² − h² 的平方根。
//
// 工具点在世界系里就是
//
//     p = O3(q1,q2) + R3(q1,q2,q3)·c
//       = O3 + h·a3 + Rot(a3, q3)·c⊥_world
//
// 其中 O3 是 joint3 原点、a3 是 joint3 轴的世界方向，两者都只依赖 (q1,q2)。
// 因为 Rot(a3,q3)·c⊥ 始终垂直于 a3、且模长恒为 |c⊥|，把上式两边减去 O3 并投影：
//
//     (p − O3)·a3 = h                       …… ①
//     |p − O3|    = |c|                     …… ②
//
// **① 和 ② 就是 (q1,q2) 要满足的两个方程**（2 元非线性系统），本文件用牛顿法解，
// 通常 2~3 步收敛到机器精度。解出 (q1,q2) 后，q3 由"绕 a3 把 c⊥ 转到目标方向"
// 直接给出：
//
//     q3 = atan2( (c⊥_world × (p − O3 − h·a3)) · a3 ,  c⊥_world · (p − O3 − h·a3) )
//
// ===========================================================================
// 五、q4（自转角）与分支
// ===========================================================================
// q4 是绕工具轴的自转，**完全不影响工具点位置**（工具点就在 joint4 轴上），所以它
// 是自由量：本文件直接把它设成调用者给的 target_roll。
//
// 分支：① ② 这个系统在不同初值下会收敛到不同解（肘上/肘下、腕部翻转），本文件
// **以 q_seed 为初值**，因此会稳定地给出"离 seed 最近的那一支"——沿轨迹逐点调用时
// 把上一点的结果当 seed，解就天然连续，不会在不同构型支之间乱跳。
//
// ===========================================================================
// 六、与数值解的分工
// ===========================================================================
//   kdl_ik.hpp          solveIkLma / solveIkNrJl  —— 通用、任意链、6 维位姿
//   本文件               solveIkAnalytic          —— 4 轴专用、4 维任务、闭式
// 两者返回结构同构（q + error_code + message），便于并排对比。
//
// ===========================================================================
// 七、把一个 6 维参考姿态投影成"绕工具轴自转"
// ===========================================================================
// 本文件的接口只吃 4 维（位置 + 自转角），但上层拿到的往往是 6 维参考位姿
// （例如"位置五次 + 姿态 slerp"造出来的笛卡尔轨迹）。这时需要一次投影：
//
//   ① makeRollFrame()  在该位形处量出工具轴与滚转参考向量；
//   ② rollAngle()      把参考姿态投到"绕工具轴"这一维上，得到自转角。
//
// 投影会**丢掉**参考姿态里"工具轴指向"那一维 —— 这是构型决定的，不是精度问题：
// 4 轴臂在给定位置能实现的姿态只有 1 维，slerp 走出来的姿态路径几乎处处落在这个
// 流形之外。拿 6 维误差去逼它，误差里有 2 维永远降不下去，数值逆解会以"梯度消失"
// 收场（实测 solveIkLma 在第 11/17/23/27 个采样点报错，而第 0 点——正好落在可达
// 流形上——是好的）。投影之后的残差有两笔账，必须分开记：
//
//   滚转残差   rollAngle(实到) − rollAngle(参考)   ← 该臂能控制，可以要求它小
//   完整夹角   GetRotAngle(参考 → 实到)            ← 含够不着的那一维，天然不为 0
//
// 把两者混在一起（只报"姿态残差"）会让任何 4 轴轨迹都判失败，也会让人误以为
// "跟踪误差 3 度"是控制器不行。详见 kdl_control 的 CartesianSpaceTask。
//
// 单位：位置 m、角度 rad，全部表达在基座坐标系。

#ifndef KDL_KINEMATICS__KDL_IK_ANALYTIC_HPP_
#define KDL_KINEMATICS__KDL_IK_ANALYTIC_HPP_

#include <string>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

namespace kdl_kinematics
{

/// 本文件实现的解析解要求链恰好是 4 个关节。
constexpr unsigned int kPlanar4ArmJoints = 4;

/**
 * @brief "平面 2R + 2 自由度腕部"构型的几何参数（全部在基座坐标系）。
 *
 * @note 这个结构体是 extractPlanar4ArmGeometry() 的输出、solveIkAnalytic() 的输入。
 *       把"提取"和"求解"分开，是因为沿一条笛卡尔轨迹逐点求逆解时要调用几百次，
 *       而几何量只需提取一次。
 * @note 所有量都由 KDL::Chain 的实际段位姿算出，**没有任何硬编码的连杆长度**——
 *       换模型后重新提取即可。这是刻意的：把几何写死在代码里，换模型时不会报错、
 *       只会静默给出错解。
 */
struct Planar4ArmGeometry
{
  /// joint1 / joint2 的公共轴方向（单位向量）。
  KDL::Vector axis;

  /// joint1 轴上的一点。
  KDL::Vector origin;

  /// 臂平面内的一组正交基：u × v = axis。
  KDL::Vector u;
  KDL::Vector v;

  /// joint1 轴 → joint2 轴的垂距（平面内），单位 m。
  double L1 = 0.0;

  /// joint2 轴 → joint3 轴的垂距（平面内），单位 m。
  double L2 = 0.0;

  /// joint3 原点的平面外坐标（相对 origin 沿 axis 的投影），单位 m。
  double out_of_plane = 0.0;

  /// q1 与平面内第一段连杆方位角之间的结构偏置，单位 rad。
  double delta1 = 0.0;

  /// q2 与平面内第二段连杆相对转角之间的结构偏置，单位 rad。
  double delta2 = 0.0;

  /// joint3 原点沿 joint3 轴到腕心的距离，单位 m。
  double wrist_h = 0.0;

  /// 工具点相对 joint3 轴的距离（即 |c⊥|），单位 m。
  double tool_perp_radius = 0.0;

  /// q=0 处、工具点相对 joint3 轴的垂直分量 c⊥ 在**基座系**里的方向（模长 = tool_perp_radius）。
  ///
  /// @note 注意 c⊥ **不在臂平面内**：实测它主要沿臂平面法向（axis）方向，只有很小的
  ///       平面内分量。所以求 q3 必须用一般的三维绕轴旋转，不能拿平面方位角近似——
  ///       后者会给出完全错误的 q3（本文件第一版就踩了这个坑）。
  KDL::Vector tool_perp_ref;

  /// 提取是否成功。
  bool valid = false;

  /// 失败原因（中文）；成功时为空。
  std::string message;
};

/**
 * @brief 从 KDL::Chain 提取"平面 2R + 2 自由度腕部"的几何参数。
 *
 * @param chain   [in]  待分析的链；关节数必须为 4。
 * @param geo     [out] 提取结果；失败时 valid == false。
 * @param message [out] 失败原因（中文）。
 * @return true 表示链的结构符合本文件要求的构型，geo 可用。
 *
 * @note 会逐条校验文件头第一节列出的三个结构判据，任一条不满足就失败并说明原因，
 *       而不是硬着头皮给一个错的几何量。这样"拿错模型来解"会立刻暴露。
 * @note 提取方式：沿链逐段复合 KDL::Segment::pose()，在 q = 0 处取各关节的原点与
 *       轴方向，再据此算出平面基、连杆长度与角度偏置。
 */
bool extractPlanar4ArmGeometry(
  const KDL::Chain & chain, Planar4ArmGeometry & geo, std::string & message);

/**
 * @brief "绕工具轴滚转"的参考系。
 *
 * @note 4 轴臂能实现的姿态只有**一维**：绕工具轴自转。要把一个参考姿态（6 维里的
 *       姿态部分）投影成这一维，需要两样东西：工具轴本身，以及一个跟着工具转的
 *       参考向量（用来量"转了多少"）。本结构体就是这两样东西。
 */
struct RollFrame
{
  /// 工具轴（基座系）；方向 = q4 增大的旋转正方向。
  KDL::Vector axis{};

  /// 垂直于 axis 的参考向量（q4 = 0 时的值，已归一化）。
  KDL::Vector w{};

  /// w 对应的是末端坐标系的哪一个基向量。
  KDL::Vector local{};

  /// 是否有效。
  bool valid = false;
};

/**
 * @brief 在给定关节角处量出滚转参考系。
 *
 * @param chain [in]  运动学链（关节数 >= 4）。
 * @param q     [in]  关节角；只有前 3 个起作用（工具轴与 q4 无关）。
 * @param out   [out] 量出的参考系；失败时 out.valid == false。
 * @return true 表示成功。
 *
 * @note **工具轴是量出来的，不是假设的**：把第 4 个关节单独转 1 rad，两次 FK 的姿态
 *       之差就是"绕工具轴转 +1 rad"这个旋转 —— 它的转轴按定义就是工具轴、转角按定义
 *       就是 1 rad。这样不依赖任何局部坐标约定（工具轴在末端坐标系里指向哪儿、正方向
 *       怎么定），也就不会因为写死一个 (0,0,1) 而在换模型时静默给出错解。
 * @note 代价是每个调用点多两次 FK（微秒量级），换来的"不可能写错"很值。
 */
bool makeRollFrame(const KDL::Chain & chain, const KDL::JntArray & q, RollFrame & out);

/**
 * @brief 某个姿态在给定滚转参考系里的滚转角 [rad]。
 *
 * @param rotation [in] 待测量的姿态（基座系）。
 * @param frame    [in] 由 makeRollFrame() 量出的参考系。
 * @return 绕 frame.axis 从 frame.w 转到该姿态参考方向的转角，范围 (-π, π]。
 *
 * @note 只取"绕工具轴"那一维：参考方向会先投到垂直于 axis 的平面上，所以参考姿态里
 *       工具轴指向的偏差**不会**污染这个角度 —— 这正是 4 轴臂任务空间降维的落点。
 *       反过来说，两个姿态的滚转角相等并不代表它们姿态相同；要量"完整差多少"，用
 *       KDL::Rotation::GetRotAngle 单独算。
 */
double rollAngle(const KDL::Rotation & rotation, const RollFrame & frame);

/**
 * @brief 解析逆解的结果。
 *
 * @note 与 IkResult（kdl_ik.hpp）同构，便于并排对比。
 */
struct AnalyticIkResult
{
  /// 解出的关节角；失败时保持调用者传入 seed 的尺寸（4）。
  KDL::JntArray q;

  /// 错误码，沿用 KDL 约定：0 = 成功，负数 = 失败。
  ///   KDL::SolverI::E_SIZE_MISMATCH     输入尺寸不对
  ///   KDL::SolverI::E_NO_CONVERGE       牛顿迭代雅可比奇异、无法继续
  ///   KDL::SolverI::E_OUT_OF_RANGE      目标超出工作空间
  ///   KDL::SolverI::E_NOT_UP_TO_DATE    几何参数无效（未成功提取）
  int error_code = 0;

  /// 可读说明（中文）。
  std::string message;

  /// 牛顿迭代实际用掉的步数（诊断用）。
  unsigned int iterations = 0;

  /// @return error_code == 0。
  bool success() const { return error_code == 0; }
};

/**
 * @brief 4 轴臂的解析逆解：由工具点位置 + 绕工具轴自转角求 4 个关节角。
 *
 * @param chain           [in]  运动学链（4 关节）。
 * @param geo             [in]  由 extractPlanar4ArmGeometry() 提取的几何（须 valid）。
 * @param target_position [in]  工具点在基座系的位置，单位 m。
 * @param target_roll     [in]  绕工具轴的自转角 q4，单位 rad。工具点位置与它无关，
 *                              所以它是自由量，直接写进解里。
 * @param q_seed          [in]  迭代初值 + 分支选择依据。沿轨迹逐点调用时传上一点的
 *                              解，解就连续；第一次调用传当前关节角。
 * @param max_iter        [in]  牛顿迭代步数上限，默认 30（实测 2~3 步即到机器精度）。
 * @return AnalyticIkResult；失败时 message 说明原因。
 *
 * @note **不需要给工具轴方向**：它由位置唯一决定（给定分支）。这是 4 轴臂任务空间
 *       只有 4 维的直接体现，也是它和 6 维数值解最本质的区别。
 * @note 求解顺序：先牛顿解 (q1,q2) 使 ① (p−O3)·a3 = h 且 ② |p−O3| = |c|；
 *       再闭式解 q3；最后 q4 = target_roll。① ② 的几何含义见文件头第四节。
 * @note 不做关节限位检查：解出来是什么就是什么，限位交给调用者（与 solveIkLma 的
 *       分工一致）。这样"目标不可达"和"可达但超行程"是两件可区分的事。
 */
AnalyticIkResult solveIkAnalytic(
  const KDL::Chain & chain, const Planar4ArmGeometry & geo,
  const KDL::Vector & target_position, double target_roll,
  const KDL::JntArray & q_seed, unsigned int max_iter = 30);

}  // namespace kdl_kinematics

#endif  // KDL_KINEMATICS__KDL_IK_ANALYTIC_HPP_
