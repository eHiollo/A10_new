# 兼容前端的关节 follower

实现位于 `kaanh_test`。`follower_aj` 使用本项目的在线关节跟踪器，保留厂商 SDK 的 WebSocket、UDP 报文封装和 `get` 原有字段。`vr_vel`、`policy` 的运动算法不变，增加了三者互斥的运行所有权。

## 启动与启停

统一从以下目录启动，默认读取同目录的 `kaanh.xml`：

```bash
cd /home/kaanh/A10_new/build/bin
sudo ./kaanhbin
```

可先只检查 XML，下面的命令不会初始化控制器、打开机器人通信接口或发送运动指令：

```bash
sudo ./kaanhbin --check-config
```

前端原有调用顺序不变：

```text
get
follower_aj
get
# 确认 follower_msg.running_state=true 且 follower_mode="Axis" 后发送目标
stop_follower
get
# 确认 follower_msg.running_state=false
```

`follower_aj` 要求控制器无故障、机器人已使能、运动状态为 `Stopped`，并且六轴具有真实、已使能、静止且有限的反馈；虚拟电机不能启动。不会自动使能。`follower_aj`、`vr_vel`、`vr`、`policy` 同时只能启动一个，需停止上一项后再切换。

启动回复表示请求已准备并提交；第一拍实时初始化成功后才显示运行。`stop_follower` 立即请求停止并回复，制动及反馈停止后才显示未运行。普通 `stop` 也能停止新 follower。重复停止可安全调用。

成功回复保持为：

```json
{"ret_code":0,"ret_context":"","ret_msg":""}
```

WebSocket 端口保持原配置；`5999` 与其他接口使用相同 Shell，支持运行中和停止期间的 `get`，不等待 follower Plan 结束。

## UDP 关节目标

沿用 SDK 的 UDP `9999` 及原有 ARIS Msg 封装，正文为：

```text
jp --motor_pos=[-117.13,-46.66,136.69,-32.32,-70.14,-13.99]
```

只接受六个有限数，表示关节 1–6 的绝对角度，单位度；内部转换为弧度。夹爪、另一机械臂和外部轴不在控制范围内。非法包或超出 MotorConfig 关节范围的包整包丢弃并计数，不覆盖最后有效目标，不逐包回复。

发包频率不控制运动速度。实时循环以 2ms 周期执行，收到新目标就从当前已接受的位置、速度继续跟踪；历史包不建立动作队列，相同目标不会重新起步。断流后完成最后目标并保持，仍处于跟随模式，直到 `stop_follower` 或故障。

协议没有序号、发送时间戳，不能识别所有迟到、乱序或跨重启到达的网络旧包。进程内旧会话快照在重新启动时会失效；停止期间接收的目标不用于后续启动。

## 在线调参

查询当前参数：

```text
follower_config
```

在停止时或运行中修改：

```text
follower_config --max_vel=0.3 --max_acc=1.5 --max_dec=1.5
```

也可在启动时指定；未显式指定的项使用当前配置：

```text
follower_aj --max_vel=0.2 --max_acc=1.0 --max_dec=1.0
```

- `max_vel`：rad/s；默认 0.3。
- `max_acc`、`max_dec`：rad/s²；默认均为 1.5。
- 均须为有限正数。公共速度参数不超过 VR 最大逐轴上限（约 0.785398rad/s），加减速度不超过约 3.490659rad/s²。
- 每轴实际速度再取与 VR 逐轴保护上限的较小值，J1–J3 约 0.654498rad/s，J4–J6 约 0.785398rad/s。查询返回实际逐轴限速。
- 调低速度后受控减速到新上限，过渡期间速度可能暂高于新工作上限；不直接截断位置或速度。
- 普通停止遵守工作减速度；独立保护器在故障时优先执行其制动策略。
- 不承诺 jerk 连续或 jerk 受限，`max_jerk` 参数明确拒绝。

配置命令通过 `ret_code` 报告错误；成功查询/设置的 `ret_context` 是包含当前诊断 JSON 的字符串。原有启停回复结构不变。

## 显式保存

只有显式指定保存才写磁盘：

```text
follower_config --save=1
follower_config --max_vel=0.2 --max_acc=1 --max_dec=1 --save=1
```

保存位置为可执行文件旁的 `follower_config.json`，不重写机器人 XML。先验证和保存成功，再发布本次修改；失败保留旧文件及旧运行配置。使用同目录临时文件、文件同步和原子替换。重启时先读取 XML 默认值，再读取已保存配置；本次进程的命令更新在此基础上生效。

文件使用版本 1，示例：

```json
{"version":1,"max_vel":0.3,"max_acc":1.5,"max_dec":1.5}
```

已保存文件损坏、参数非法或不可读时，控制器报告配置错误并禁止启动 follower，不默默使用其他速度。修正/恢复该文件后重启控制器；移走保存文件则恢复 XML 默认值。

## 查询与诊断

`get` 的原字段 `follower_msg.running_state`、`follower_msg.follower_mode` 保留。新增 `follower_control` 包含：

- `state`：Stopped / Starting / Running / Stopping / Faulted。
- `max_vel/max_acc/max_dec`、`effective_max_vel`。
- `config_version` 和 `applied_config_version`：配置发布与实时应用版本；成功回复后可查询确认已被应用。
- `session`、目标年龄、接收/拒绝/忽略包计数、故障和配置错误。
- `settled`：指令已收敛，且反馈速度和跟踪误差满足停止判据；这不是逐包到位确认。

停止完成不要求到达最后目标，不能将 `running_state=false` 解释成目标已到位。

## 构建与离线验证

在 kaanh_test worktree 下：

```bash
cmake -S . -B build/integration -DA10_BUILD_FOLLOWER_TESTS=ON -DA10_BUILD_STARTUP_LOGIN_TEST=ON
cmake --build build/integration -j 4
ctest --test-dir build/integration --output-on-failure
```

测试覆盖跟踪连续性、重复/突发目标、降速和反向、停止、协议校验、保存/加载、并发快照，以及实际 SDK Plan 对六轴模拟反馈的完整启停。SDK 测试不初始化总线或启动真实控制器。XML 测试通过程序的 `--check-config` 路径验证。

离线验证不代表实机效果已验收；下一次启动后先检查 get、参数和小范围跟踪，再由用户确认实际效果。
