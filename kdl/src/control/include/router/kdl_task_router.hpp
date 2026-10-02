// Copyright (c) 2026, kdl_control authors.
// 教学用途：任务调度路由——按任务信号把请求分派给具体任务实现。
//
// ===========================================================================
// 为什么要有"路由"这一层
// ===========================================================================
// 如果没有它，调用方就得自己写一堆 if/else：判断任务类型、检查尺寸、检查上下
// 文、算力矩前馈、收尾……而且每加一个任务就要把这些重复一遍。路由把"每个任务
// 都要做的事"抽成三件横切工作：
//
//   1) 分派：按 req.type 找任务实现；没注册就老实返回"未实现"，不崩溃；
//   2) 统一校验：上下文是否装配好、状态向量长度是否匹配；
//   3) 统一收尾：动力学前馈与力矩上限检查（两条任务产出的都是关节轨迹，
//      所以这一段的代码与任务无关）。
//
// 具体的"怎么解"仍然只写在 tasks/ 里，路由不认识任何算法细节。
// ===========================================================================

#ifndef KDL_CONTROL__KDL_TASK_ROUTER_HPP_
#define KDL_CONTROL__KDL_TASK_ROUTER_HPP_

#include <map>
#include <memory>
#include <string>

#include "router/kdl_task_base.hpp"

namespace kdl_control
{

/**
 * @brief 任务路由：注册任务实现，并把一次任务信号解算成关节空间轨迹。
 */
class TaskRouter
{
public:
  /**
   * @brief 构造路由，并注册两个内置任务（关节空间 / 笛卡尔空间）。
   * @note 构造出来的路由上下文是空的，必须先 setContext() 或
   *       loadContextFromUrdf() 装配，否则 dispatch() 会直接返回失败。
   */
  TaskRouter();

  /// 析构：在 .cpp 里定义，这样公共头不必 include 任何具体任务的头文件。
  ~TaskRouter();

  /// 禁止拷贝（持有任务实例与上下文，拷贝没有语义）。
  TaskRouter(const TaskRouter &) = delete;
  TaskRouter & operator=(const TaskRouter &) = delete;

  // ---- 上下文 ----
  /// 设置机器人上下文。
  void setContext(const RobotContext & ctx);

  /// @return 当前机器人上下文（只读）。
  const RobotContext & context() const { return ctx_; }

  /**
   * @brief 从 URDF 文件一次性装配上下文（链 + 关节行程 + 力矩上限）。
   * @param urdf_file [in]  URDF 路径。
   * @param ctx       [out] 装配结果；失败时被清空。
   * @param message   [out] 失败原因（中文）。
   * @param base_link [in]  链的起点 link，默认 "base_link"。
   * @param tip_link  [in]  链的末端 link，默认 "link6"。
   * @return true 表示装配成功。
   *
   * @note 这两个默认值与 kdl_tools::kDefaultBaseLink / kDefaultTipLink 一致；这里
   *       写成字面量是为了不让公共头把 <urdf/model.h> 带进来。

   *
   * @note 速度/加速度/jerk 上限与笛卡尔限位**不在这里设置**：URDF 里没有 jerk
   *       上限，速度上限是否启用也取决于使用场景，所以由调用者显式填 ctx 的
   *       两个 limits 字段（"长度 0 / 值 <= 0 = 不校验"）。
   */
  static bool loadContextFromUrdf(
    const std::string & urdf_file, RobotContext & ctx, std::string & message,
    const std::string & base_link = "base_link", const std::string & tip_link = "link6");

  // ---- 任务注册 ----
  /// 注册/替换一个任务实现（后续任务的扩展点）。
  void registerTask(std::shared_ptr<TaskBase> task);

  /// @return 该类型是否已注册。
  bool hasTask(TaskType type) const;

  /// @return 该类型对应的任务名；未注册时返回 "unimplemented"。
  const char * taskName(TaskType type) const;

  // ---- 主入口 ----
  /**
   * @brief 按任务信号分派并解算。
   * @param req       [in] 任务信号（含类型、目标、时间参数与开关）。
   * @param q_now     [in] 当前关节角 [rad]。
   * @param qdot_now  [in] 当前关节速度 [rad/s]（长度可为 0，表示未知）。
   * @return 解算结果；失败时 message 说明原因。
   *
   * @note 三件事都在这里做完：分派 → 校验 → 解算 → 可选的动力学收尾。
   */
  ControlResult dispatch(
    const TaskRequest & req, const KDL::JntArray & q_now, const KDL::JntArray & qdot_now) const;

  /**
   * @brief 动力学可用性自检：链上到底有没有惯量参数？
   * @param ctx     [in]  机器人上下文。
   * @param message [out] 失败原因（中文）。
   * @return true 表示存在非零的惯性耦合（即 link 上确实有质量/惯量）。
   *
   * @note 判据刻意选了"给一个 1 rad/s² 的假想加速度，看 τ = M(q)·q̈ 是否为 0"：
   *       KDL 的 ChainIdSolver_RNE 在段没有惯量参数时会安静地返回全零，而
   *       **只看重力项会误判**（例如全是竖直轴的 SCARA，重力项本来恒为 0，
   *       但它的惯量矩阵并不为零）。所以在启用力矩前馈前先过这一关，比事后
   *       对着一堆 0 排查要省事。
   */
  static bool checkDynamicsAvailable(const RobotContext & ctx, std::string & message);

private:
  /**
   * @brief 横切步骤：按 sample_dt 采样轨迹，逐点调用逆动力学填力矩前馈，
   *        并按 req.check_torque_limit 判定力矩是否超限。
   * @param req     [in]     任务信号（取 sample_dt / check_torque_limit）。
   * @param result  [in,out] 解算结果（轨迹已填好；本函数填 torque_* 字段）。
   * @param message [out]    失败原因。
   * @return true 表示前馈计算完成且（若要求检查）力矩未超限。
   */
  bool computeTorqueFeedforward(
    const TaskRequest & req, ControlResult & result, std::string & message) const;

  RobotContext ctx_{};
  std::map<TaskType, std::shared_ptr<TaskBase>> tasks_{};
};

}  // namespace kdl_control

#endif  // KDL_CONTROL__KDL_TASK_ROUTER_HPP_
