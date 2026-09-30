/**
 * @file pinocchio_parser.hpp
 * @brief 把 URDF 文件解析为 Pinocchio 模型的最小工具集（pinocchio_tools / Tool 模块）。
 *
 * @details 本模块只专注一件事：解析。它负责
 *          - 读取磁盘上的 URDF 文件；
 *          - 调用 Pinocchio 的 urdf 解析器生成 pinocchio::Model（关节、连杆惯性、限位等）；
 *          - 为模型配套创建 pinocchio::Data（后续算法所需的数据缓冲区）；
 *          - 汇总便于上层直接使用的元信息（模型名、关节名、帧名、自由度维数等）。
 *
 *          解析失败时本模块不抛异常，而是通过 RobotModel::valid / RobotModel::message 反馈，
 *          方便在 ROS 节点中做统一的错误处理。
 *
 * @note 依据 Pinocchio 官方建议，pinocchio/fwd.hpp 必须是第一个被包含的 Pinocchio 头文件，
 *       否则会因 Boost variant 尺寸不一致而产生难以理解的编译错误。
 */

#pragma once

// 必须位于所有其他 Pinocchio 头文件之前。
#include "pinocchio/fwd.hpp"

#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/multibody.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace pinocchio_tools
{

/**
 * @brief 根关节类型，决定模型是固定基座还是浮动基座。
 */
enum class RootJointType
{
  /// 固定基座：URDF 的根连杆被视为世界坐标系，模型只包含机器人本体的自由度。
  kFixed = 0,
  /// 浮动基座：在根节点插入一个 6 自由度浮动关节（3 平移 + 3 旋转）。
  kFreeFlyer = 1,
};

/**
 * @brief parseUrdf() 的可选参数。
 *
 * @note 所有字段都有合理默认值，调用方按需覆盖即可。
 */
struct ParseOptions
{
  /// 根关节类型，默认固定基座。
  RootJointType root_joint_type = RootJointType::kFixed;

  /// 当 root_joint_type 为 kFreeFlyer 时，插入的浮动关节名称。
  std::string root_joint_name = "root_joint";

  /// 是否让 Pinocchio 打印解析过程信息（调试用）。
  bool verbose = false;

  /// 是否把 URDF 中的 mimic 关节按 mimic 语义解析。
  bool mimic = false;
};

/**
 * @brief 一次 URDF 解析的完整结果：模型 + 数据 + 元信息。
 *
 * @details 该结构体把 Pinocchio 的 Model / Data 与常用的查询元信息打包在一起，
 *          使调用方无需重复书写遍历代码即可拿到关节列表、帧列表等信息。
 *          Data 在解析成功后即与 Model 匹配，可以直接交给
 *          pinocchio::forwardKinematics 等算法使用。
 */
struct RobotModel
{
  /// Pinocchio 运动学 / 动力学模型（由 URDF 解析得到）。
  pinocchio::Model model;

  /// 与 model 配套的数据缓冲区，用于存放算法中间结果与输出。
  pinocchio::Data data;

  /// 模型名，取自 URDF 根节点 <robot name="...">。
  std::string name;

  /// 本次解析所使用的 URDF 文件路径。
  std::string source_path;

  /// 解析结果的说明信息：成功时为 "success"，失败时为具体原因。
  std::string message;

  /// 解析是否成功。为 false 时其余字段不可用于计算。
  bool valid = false;

  /// 是否为固定基座（true）或浮动基座（false）。
  bool fixed_base = true;

  /// 除 universe 之外的全部关节名，顺序与 Pinocchio 关节索引一致。
  std::vector<std::string> joint_names;

  /// 全部帧名，顺序与 Pinocchio 帧索引一致。
  std::vector<std::string> frame_names;

  /**
   * @brief 广义位置向量维数（configuration dimension）。
   * @return model.nq。
   */
  int nq() const { return model.nq; }

  /**
   * @brief 广义速度向量维数（tangent dimension）。
   * @return model.nv。
   */
  int nv() const { return model.nv; }

  /**
   * @brief 关节总数（含 universe 伪关节）。
   * @return model.njoints。
   */
  int numJoints() const { return model.njoints; }

  /**
   * @brief 关节数量（不含 universe 伪关节）。
   * @details 浮动基座解析时该计数包含插入的根关节，因此它表示的是
   *          「模型中的显式关节数」而非「驱动关节数」。
   * @return joint_names 的长度。
   */
  int numJointsExcludingUniverse() const { return static_cast<int>(joint_names.size()); }

  /**
   * @brief 判断模型里是否存在指定名称的关节。
   * @param[in] joint_name 待查询的关节名。
   * @return 存在返回 true，否则返回 false。
   */
  bool hasJoint(const std::string & joint_name) const { return model.existJointName(joint_name); }

  /**
   * @brief 查询指定名称关节的索引。
   * @param[in] joint_name 待查询的关节名。
   * @return 关节索引；不存在时返回 -1。
   */
  int jointId(const std::string & joint_name) const
  {
    return model.existJointName(joint_name) ? static_cast<int>(model.getJointId(joint_name)) : -1;
  }

  /**
   * @brief 判断模型里是否存在指定名称的帧。
   * @param[in] frame_name 待查询的帧名。
   * @return 存在返回 true，否则返回 false。
   */
  bool hasFrame(const std::string & frame_name) const { return model.existFrame(frame_name); }

  /**
   * @brief 查询指定名称帧的索引。
   * @param[in] frame_name 待查询的帧名。
   * @return 帧索引；不存在时返回 -1。
   */
  int frameId(const std::string & frame_name) const
  {
    return model.existFrame(frame_name) ? static_cast<int>(model.getFrameId(frame_name)) : -1;
  }
};

/**
 * @brief 解析 URDF 文件并生成 RobotModel。
 *
 * @details 该函数是 Tool 模块的唯一入口：
 *          - 校验路径有效性；
 *          - 依据 @p options 选择固定基座或浮动基座方式调用
 *            pinocchio::urdf::buildModel；
 *          - 解析成功后创建配套的 Data 并汇总元信息；
 *          - 任何失败都会被捕获并写入返回值，不向外抛异常。
 *
 * @param[in] urdf_path URDF 文件的完整路径。
 * @param[in] options   解析选项，默认固定基座、非 verbose。
 * @return 解析结果。调用方应先检查 RobotModel::valid，
 *         失败时通过 RobotModel::message 获取原因。
 *
 * @note 解析只构建运动学 / 动力学模型，不会加载 visual / collision 网格，
 *       因此 URDF 中形如 package:// 的网格路径不需要真实存在。
 *
 * @see RobotModel, ParseOptions
 */
RobotModel parseUrdf(const std::string & urdf_path, const ParseOptions & options = ParseOptions());

}  // namespace pinocchio_tools
