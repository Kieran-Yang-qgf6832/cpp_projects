# ControlStatus.msg
# ---------------------------------------------------------------------------
# 力矩控制器（kdl_effort_controller）执行状态的实时回报。
#
# 为什么需要它：控制节点（kdl_control_node）承接的 service 语义是"把这条轨迹
# 执行完再返回"，但推进参考轨迹、施加力矩的是控制器（在 CM 的 update() 里）。
# 控制器没有 action server，节点只能靠一个状态话题知道"什么时候开始/结束"。
#
# 发布：控制器经 realtime publisher 以 status_publish_rate（默认 100 Hz）发到
#       ~/status（即 /kdl_effort_controller/status）。
# 消费：kdl_control_node 订阅它，实现 active:true→false 的等待状态机。
# ---------------------------------------------------------------------------

std_msgs/Header header

# 是否正在执行一条轨迹（收到新轨迹置 true，跑完置 false）。
# 空闲/锁位（重力前馈保持）时为 false。
bool active

# 控制器自身的错误码：0 = 正常；>0 = 内部错误（见 message）。
# 与 ControlTask.srv 的三段语义互补：这里只报"控制器侧"的错误。
int32 error_code

# 中文说明（当前状态或错误原因）。
string message

# 当前参考轨迹的执行进度 [0, 1]；空闲时为 1.0。
float64 progress

# 当前参考轨迹的总时长 [s]；空闲时为 0.0。
float64 duration
