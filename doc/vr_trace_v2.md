# VR trace v2：姿态与时序诊断

## 本轮限位变更

`kaanh.xml` 的第四个 MotorConfig（slave=3）负向限位按用户要求扩大至 −250°（−4.363323129985824 rad），正向仍为 +175°，其余关节不变。这是配置范围，不代表已经测得机械极限。

`vr_vel` 启动时从六个 Motor 的 `minPos()/maxPos()` 读取 XML 生效范围，启动检查、IK 后保护及预留制动距离均使用同一组上下限。启动行会打印 `j4_min=-4.36332 j4_max=3.05433`，每行 trace 也记录各轴范围。仍预留制动距离与 0.002 rad 余量。速度、加速度和故障锁存保护保持原有行为。

重新启动程序以加载 XML 和新二进制，再运行带 `--joint_diag=1` 的 `vr_vel`。日志为原格式的追加扩展：旧字段名称、顺序不变，追加 `trace_version=2` 和下列字段。

## 时序与包状态

所有 `*_ns` 字段单位为整数纳秒，分析时用整数读取，避免将绝对时间戳先转换为低精度浮点数。

| 字段 | 含义 |
| --- | --- |
| `control_time_ns`, `control_dt_ns` | 机器人 steady_clock 下的 executeRT 入口时间及与上一入口的真实间隔；第一行间隔为 0。旧 `actual_dt` 仍是计数×2 ms，并不是真实测量值。 |
| `packet_updated`, `packet_result` | 本周期是否消费新邮箱内容，以及处理结果：none/blocked/inactive/anchored/updated/duplicate/stale。anchored 表示协议建锚，仍需同时检查 fault，控制器可能拒绝尚未静止时的建锚。 |
| `mailbox_seq`, `mailbox_skipped` | 接收邮箱版本号、本次消费之间被最新包覆盖的版本数；首次消费不计此前跳过量。覆盖不等于网络丢包。 |
| `robot_receive_time_ns` | 包含该行最终换行符的 recv 返回后时间。同一 recv 中的多行共用时间，可观察批量到达。不是网卡硬件接收时间。 |
| `robot_publish_time_ns` | 解析完成、取得邮箱锁后准备发布包的时间。 |
| `packet_consume_time_ns` | 控制线程成功取到邮箱内容后的时间。 |
| `rx_anchor_id`, `rx_sample_sequence`, `rx_active` | 最新被消费原始包的状态，包括被拒绝的包；原有 anchor_id/sample_sequence 表示协议影子状态。 |
| `client_sample_time_valid`, `client_send_time_valid` | 发送端是否提供对应时间戳。缺失时为 0，不能将对应的零时间戳用于计算。 |

没有新包的周期仍保留最近一次包的时间戳和原始输入，`packet_updated=0`、`packet_result=none`。按包统计时只取 `packet_updated=1`；按目标更新统计时再区分 updated/anchored、duplicate 和 blocked 等。

机器人本机可直接计算：

- 接收线程处理/排队耗时：`robot_publish_time_ns - robot_receive_time_ns`。
- 邮箱到控制线程耗时：`packet_consume_time_ns - robot_publish_time_ns`。
- 接收至消费总耗时：`packet_consume_time_ns - robot_receive_time_ns`。
- 本机接收间隔和消费间隔，以及 `control_dt_ns` 是否偏离 2 ms。

## 姿态与限制阶段

- `rx_offset_x/y/z`（m）、`rx_offset_rx/ry/rz`（旋转向量 rad）：最新被消费包的原始六维 offset，不是 Euler 角。
- `robot_anchor_r00..r22`、`robot_anchor_x/y/z`：建锚位姿。
- `user_target_r00..r22`：影子协议应用 offset 后的目标姿态。
- `reference_r00..r22`：参考生成器本周期输出姿态。
- 原有 `actual_r*` 和 `command_r*`：实际 FK 和最终下发关节位置的 FK。
- `requested_v_*`, `requested_w_*`：P 控制计算后的速度，尚未经过笛卡尔限速和加速度限制；停车时为零。
- `slewed_v_*`, `slewed_w_*`：笛卡尔限速/加速度处理后、进入 IK 前的速度。
- 原有 `v_cmd_*`, `w_cmd_*`：原逻辑中的执行速度状态，关节保护介入后会由接受的 FK 位姿增量回算。
- `kp`, `kp_rot`, `vmax`, `wmax`, `ref_vmax`, `ref_wmax`, `amax`, `jacc`, `joint_amax` 及各轴 `jN_min_pos/max_pos/max_speed`：本轮真实生效参数。笛卡尔速度上限仍分别取 vmax/ref_vmax 与 wmax/ref_wmax 的较小值。

对姿态差使用相对旋转的角度，不能直接把两个旋转矩阵逐元素相减当成角度。原 `track_rot` 是参考生成器推进前测得的跟踪误差，新增 reference 矩阵是推进后的值，两者会有一个参考步长的差异。

## VR 发送端的可选字段

旧包继续兼容。发送端可在原 SET_EE_ANCHOR JSON 中增加：

```json
{
  "client_sample_time_ns": 1234567890123456,
  "client_send_time_ns": 1234567891123456
}
```

这只是新增字段示例，原来的 active/session_id/anchor_id/sample_sequence/offset/gripper 仍需发送。两时间戳应使用发送端同一个单调时钟：前者在 VR 样本采集时记录并随样本传递，后者在发送前记录；不要都在发送时临时填写。Python 可使用 `time.monotonic_ns()`。字段必须是非负整数，若同时提供则 send 不得早于 sample。机器人侧 receive/publish 字段不接受客户端赋值。

本仓库没有当前 VR 采集/发送端实现，因此本轮只增加协议接收和记录支持；VR 端未接入前 valid=0。客户端同一时钟下的 send−sample 可以反映发送前积压。两台机器的单调时钟不能直接相减得到单向网络延迟，需要额外时钟同步或往返测量。即使有这些时间戳，也不能把接收间隔直接命名为网络延迟。

日志字段增多，应同时检查实际 `control_dt_ns`，确认诊断输出未造成明显周期抖动。本轮离线测试和编译不代替新配置的实机验证。
