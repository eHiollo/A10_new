# client（VLA / OpenPI 桥接）

## kaanh 整合版（2026-10-15）

保留 2026-10-09 RTC 实机默认值和 async/sync/pipeline。日志由控制器
`policy --log=1`（默认）或 `policy --log=0` 控制；bridge 常驻期间，每次
policy 启动创建独立日志目录，停止后关闭文件。`--log-dir` 指定根目录。
日志保持 RTC 的 `events.jsonl`、`bridge.log` 和首次观测图像格式，继续使用
`python client/analyze_policy_log.py` 分析。旧控制器缺少日志状态字段时不启用文件日志。

启动控制器会尝试 Manufacturer 自动登录；失败保留原有诊断，不输出密码。
动态密码工具参见 [登录说明](KAANH_PASSPORT.md)。本轮未整合 TTC 多候选验证和旧 VR 末端接口。


## 当前固定配置（2026-10-09）

按用户确认，固定13:54第二轮实机参数为默认值，只更改默认配置，不更改轨迹或融合算法：

- 控制器：`arm_vel=0.3rad/s`、`arm_acc=1.5rad/s²`、`future_handoff=1`、`jerk_smooth=0`；夹爪50mm/s、位移阈值和独立VR关节保护不变。
- bridge：`mode=async`、`async_smoothing=true`、`temporal_ensemble=true`、`ensemble_decay=0.01`、`ensemble_gripper=true`、`buffer_target=1`；即手臂和夹爪均融合，仍按旧预测优先加权。
- 相机：顶部`2`不旋转、右侧`0`旋转180°，threaded取图、640×480；policy服务`192.168.110.119:8000`，机器人服务`127.0.0.1:8080`。

先`stop`并等待制动完成，再重启编译后的控制器和bridge以加载默认值；不要在运动中切换进程，也不要同时启动两个bridge。之后bridge可直接启动：

```bash
conda activate client
cd /home/kaanh/A10_new/client
python run_bridge.py
```

控制器直接输入`policy`，等价于`policy --arm_vel=0.3 --arm_acc=1.5 --future_handoff=1 --jerk_smooth=0`。旧命令里的显式参数仍会覆盖新默认，需去掉不再需要的慢速、旧衔接或jerk参数。bridge启动时配置会完整写入session日志。

需要关闭融合对照时用`--no-temporal-ensemble`，不是简单去掉`--temporal-ensemble`。`--no-ensemble-gripper`只关闭夹爪融合。单独指定`--mode sync`或`--mode pipeline`时默认不融合，保留原模式行为；显式要求这些模式同时开启融合仍会报错。`--hz`默认10，仅影响sync整轮最小周期，不改变async动作步长。

固定参数不代表所有任务已验收；保持同模型、相机和起点复测对准成功率，日志正常或较少缓接不能替代实机定位精度检查。

以下为开发沿革，旧默认值、旧启动命令和当时的融合开关状态均以本节为准。

## 历史试验：加速度衔接与 jerk 限制（2026-10-09）

为保留已经实机验证的对照，直接输入 `policy` 仍使用下文三次轨迹。新增五次轨迹仅在控制器输入 `policy --jerk_smooth=1` 时启用，且只作用于 `future_handoff=1`、bridge 开启 async_smoothing 的异步批次。`arm_jerk` 默认20rad/s³是本次试验的指令变化率上限，不是已标定的硬件极限；速度、工作加速度1.0rad/s²、夹爪50mm/s和独立关节保护均未提高。

新轨迹继承实际已下发的位置、离散速度和离散加速度，以过去两个2ms位置点约束段首，检查包含这两个历史点的完整多项式速度/加速度/jerk，避免只继承有限差分速度造成第一拍加速度折半。终点连续导数加速度设为零；跟随末速度由相邻预测动作给出。拒绝会额外绕多次往返的曲线；不能满足约束时先尝试延长跨度，仍失败则拒绝本包并继续旧有效段，不暗中降级回三次曲线。旧段结束但仍有末速度时另规划有界制动段，零末速度段结束后保持。它约束的是500Hz名义时钟下的关节位置指令差分，不代表实测机械加速度或振动已被保证消除，夹爪没有新增 jerk 约束。

切换前 `stop` 并等待制动完成，再重启新编译控制器；不要在运动中替换进程。bridge 可继续常驻，保持 `--top-camera 2 --right-camera 0` 和融合关闭。重启后先在无障碍区域用 `policy --jerk_smooth=1` 短程确认；每次切换均停止并等制动完成，对照命令为 `policy --jerk_smooth=0`。sync/pipeline 保持原路径。普通 stop、保护器干预和故障优先保证原有制动，不承诺这些情况满足工作 jerk 限制；新模式遇到保护器修正会记录 `trajectory_continuity_interrupted` 并停止，不继续使用已失去初值一致性的曲线。

实机核对项已加入日志：`sample.jerk_limited`、`arm_jerk`、`max_command_jerk_rad_s3`、`command_acceleration_rad_s2`，以及每2ms收集、每100ms汇总的六轴 `max_following_error_rad`。原三次对照也记录这些测量。状态列出两种可用规划器，批次选择五次曲线后 `async_handoff_planner=quintic_discrete_pva_v1`；实际运行以采样事件为准，刚替换/拒绝的批次状态不代表仍在执行的旧段。重点比较 jerk、关节误差、控制耗时、`jerk_plan_rejected`/回退频率、任务耗时和抓放时机，不能只看 jerk 降低。更严格的衔接约束可能增加缓接和响应滞后，目前尚未实机验收。

## 历史：可达性规划衔接（2026-10-09）

平滑 async、`future_handoff=1` 现在使用同一条三次轨迹做衔接检查与逐拍输出：从上一拍保护器已接受的位置、速度出发，检查整段关节速度/加速度和夹爪50mm/s限速，再选择预测时间范围内可按时到达的目标。连续同向的后续动作保留末速度，保持/反向/最后一帧以零末速度衔接；不重复输出段首，不再叠加旧的位置追踪滤波。默认关节上限不变，工作加速度仍为1.0rad/s²。

**没有可达目标时不再硬接actions[29]。** 改为缓接首个未过期的有效动作，不额外丢掉整个尾段，并记录`handoff_fallback=true`。新批次过期或无有效动作时，从当前运动状态缓接上一段已接受的目标；后续无动作则受控减速，不跳回旧目标。独立VR关节保护、普通stop制动及位置/跟踪误差检查保留。没有移植VR的笛卡尔包络，也没有模型级RTC动作前缀条件化。

停机并等待制动结束，重启新编译的`build/kaanh_rtc/bin/kaanhbin`；bridge可以继续常驻，保持正确相机`--top-camera 2 --right-camera 0`，不要加`--temporal-ensemble`。控制器直接输入`policy`，不需要新增启动参数。先在无障碍范围短程检查再试任务；编译/离线回归不等于实机验收。需要对照此前的线性参考+滤波，用`policy --future_handoff=0`；sync/pipeline及关闭async_smoothing的路径不启用新规划器。

状态`async_handoff_planner=cubic_pv_v1`标识本次构建能力，实际是否使用看`batch_planned_trajectory`/事件`planned_trajectory`。新日志包含`command_velocity_rad_s`、`handoff_fallback`、`active_plan_remaining_s`和`target_error_rad`。`handoff_late_s`是初始规划的名义迟到量；保护器后续干预会单独记录`trajectory_guard_replan`。`reference_lag_rad`现在主要反映已规划轨迹与保护器输出的偏差，不能将其接近零当成模型/任务跟踪误差为零。新async不再输出错误的整队线性ETA，`remaining_time_s=-1`表示未知；队列为空时仍须结合`motion_settling`/`idle`判断，sync/pipeline的ETA规则不变。

以下保留开发沿革，早期参数和插值方式不代表当前默认行为。

## 历史：关节限速对齐 VR 关节层

最新平滑试验：速度上限继续采用 VR 的逐轴上限，但日常平滑器的默认 `arm_acc` 改为 `1.0rad/s²`，不再每次换段都尽快冲到 VR 硬上限 3.4907。保护器的制动/故障加速度上限仍为 3.4907。若当前控制器已经包含 VR 关节保护，无需重启即可在停止上一条 policy 后使用 `policy --arm_acc=1.0` 作同等试验；新默认值需重启控制器加载。此调整只作用于启用 async_smoothing 的手臂滤波，不能把模型本来要求保持姿态的阶段强行变成运动。

用户确认采集使用无额外参数的 `vr_vel`。policy 当前默认复用 `VrJointGuard` 的关节包络：J1–J3 为 0.6545rad/s，J4–J6 为 0.7854rad/s，加速度 3.4907rad/s²，夹爪限速降为 50mm/s。读取 MotorConfig 的实际位置范围，增加 0.1rad 跟踪误差、逐拍跳变及持续限幅故障保护；故障停发新动作并受控制动，需检查后重新启动 policy。正常 stop 也改为受控制动，不是急停。

**仅对齐关节层，不是完整 VR 运动约束。** VR 另有末端 0.12m/s、0.32rad/s 限速及笛卡尔加速度、参考轨迹和锚点保护，policy 尚未复用。相同关节上限不保证相同末端速度或任务安全，日志 `policy_cartesian_limits_applied=false` 明确标记这一点。不要据此直接在障碍物附近跑完整任务。停止机器人后重启控制器，先在无障碍环境短距离检查；保守回退用 `policy --arm_vel=0.075 --arm_acc=0.5`，仍保留新关节保护。

`arm_vel` 是六轴公共上限，实际逐轴再取与 VR 上限的较小值；`arm_acc`、`grip_vel` 也不会超过上述 VR 关节层/夹爪上限。源 XML 与构造函数已同步，旧 XML 中已有的较低参数不会被强制提高。下文 0.075/0.05 为历史试验记录。

当前试验默认手臂限速提高到 `0.075rad/s`（此前 `0.05`，提高 50%）；加速度仍为 `0.5rad/s²`，夹爪限速、动作步长与平滑不变。源 XML 和构造函数均同步。停止机器人后重启控制器生效；需要退回原速度时在控制器输入 `policy --arm_vel=0.05`。模型输出不自带遥操作速度上限，提速后需重新检查跟踪误差、回拉和抓取/释放时机。下文早期记录中的 0.05 是上一版基线。

## 历史：ACT 式融合试验（当时默认关闭）

当前稳定版已提交 `a9b0aaa`。停止机器人后重启 bridge，在原命令后加 `--temporal-ensemble` 即可尝试，控制器仍输入 `policy`，无需为本次试验重新编译 C++。去掉该参数即可回到稳定版行为。

按观测时间 `t_obs + i/30` 对齐新旧预测，非整数索引用线性插值，只融合时间覆盖范围内的原始预测，不外推、不递归平均。权重参考 [ACT 官方实现](https://github.com/tonyzhaozh/act/blob/main/imitate_episodes.py)：候选从旧到新排列，使用 `exp(-k * rank)`，默认 `--ensemble-decay 0.01`，因此旧预测略占优；这不是“越新权重越大”。零值动作也有效，不用零值判断缺失。

默认七维都融合；加 `--no-ensemble-gripper` 可只融合手臂。夹爪平均可能造成半开或开闭延后，必须对照任务成功率。保留原有限速、加速度和延迟感知衔接。融合基于名义预测时间，不是严格按实际执行进度的原版 ACT 逐拍集成；受限速影响仍有时序偏差，本试验尚未实机验证。

停机、队列代际变化或发送异常会清空融合历史；历史最多 32 批，过期覆盖自动移除。新增 `temporal_ensemble` 日志包含原始/融合动作、每步参与批次数、权重配置和改变量，可与原 `inference`/`batch_sent` 对照。

精细操作第二阶段：平滑 async 的新批次首段额外采用观测年龄作为最低衔接跨度（通常约 0.3 秒），减轻快速追逐旧目标造成的回拉；同批后续动作保持原步长下限。手臂/夹爪限速不变，sync/pipeline 不变。开闭夹爪可能更慢，需要检查释放时机和成功率；停止机器人后重启控制器生效，bridge 可继续常驻。

精细动作时序修复：默认平滑 async 不再按最快 4ms 消费一个小动作，而是至少保留名义 1/30 秒插值跨度（500Hz 量化后每段占用 36ms）。手臂/夹爪共用段时间，原速度限制仍有效；长距离允许更慢。此改动需重启控制器，bridge 启动命令不变，实际往返改善需下一次实机验证。

async 默认启用加速度受限衔接（需要重启新编译的 `build/kaanh_rtc/bin/kaanhbin`）。控制器仍输入 `policy`，默认手臂限速 0.05rad/s 不变，新增 `arm_acc=0.5rad/s²`；bridge 加 `--no-async-smoothing` 可作原插值对照，sync/pipeline 保持原行为。平滑会增加响应滞后，实机效果仍需验证；切换前先停机。日志新增指令速度、加速度和参考滞后，可用下一轮任务检查。

```bash
cd /home/kaanh/A10_new/client
python run_bridge.py --policy-host 192.168.110.119 --policy-port 8000 \
  --robot-host 127.0.0.1 --robot-port 8080 \
  --mode async --hz 10 --top-camera 2 --right-camera 0
```

2026-10-08：bridge 支持常驻，`policy` / `stop` 可独立启停机械臂；相机年龄仅记日志。新日志自动写入 `client/logs/run_*/`，用 `python client/analyze_policy_log.py` 汇总。实机证据、日志字段和待确认项见 [异步推理实机记录](../doc/async_policy_field_notes.md)。`--hz` 仅用于 sync，不影响异步发送频率或模型动作步数。

同步运行“一段一停”时，可测试 `--mode pipeline`：尾段提前推理，旧 batch 执行完再衔接。需重启本轮新编译的控制器以提供 `remaining_time_s`；初始提前量为 `--prefetch-lead 0.45`，随后按推理耗时调整。参数传给 `python run_bridge.py`，控制器命令仍为 `policy`。

把 OpenPI 推理服务（websocket）与本仓库 `a10_tcp_server.cpp` 的机器人 TCP 服务桥接起来。  
原独立目录：`/home/kaanh/Desktop/kannhbin_client`，已并入本仓库 `client/`。

## 已实现能力

- 对接 OpenPI websocket 推理服务（`msgpack + numpy` 协议）
- 对接本仓库 `a10_tcp_server.cpp` 行分隔 TCP 协议
  - 请求机器人状态：`GET_FOLLOWER_STATE`
  - 单行即时目标（与策略 batch 无关）：`{"q":[...]}\n` → `target_q_`
  - **策略轨迹（多步）**：`SET_JOINTS_BATCH {"actions":[[7 floats],...]}\n` → 服务端入队 `target_q_batch_` 并**立即回一行** `{"accepted":true,"queued":T}\n`（失败为 `accepted:false`）
  - 策略是否执行完：`GET_POLICY_STATUS` → `{"idle":bool,"remaining":int}`（`remaining` 为队列中尚未执行的步数）
- 从 USB 摄像头读取图像，用 `openpi_client.image_tools.resize_with_pad` 等比缩到 224×224 再补黑边（不直接拉伸），保持 RGB、HWC、uint8
  - `observation/state`: `float32 (7,)`，关节绝对角 + 夹爪绝对开度（mm，约 0–85）
  - `observation/images/right`: `uint8 HWC` RGB，224×224
  - `observation/images/top`: `uint8 HWC` RGB，224×224；不接顶视相机时不发这个键
- 组包观测并发起推理，经 **`SET_JOINTS_BATCH`** 下发
  - `--mode async`（默认）：执行中继续推理，下一段到达后替换还没走完的队列
  - `--mode sync`：等 `GET_POLICY_STATUS` 的 `remaining==0` 再采下一次观测
  - `STOP_POLICY`：清空队列并让 `policy` 下一拍退出，停在当前位置
- 提供摄像头抓图脚本，按端口号命名图片（用于区分相机）

## 安装

本机已有 conda 环境 `client`（含 opencv / websockets 等），推荐：

```bash
conda activate client
cd /home/kaanh/Desktop/kaanhbin_wjy/client
# 若换机器：python -m pip install -r requirements.txt
```

或：

```bash
cd /home/kaanh/Desktop/kaanhbin_wjy/client
python3 -m pip install -r requirements.txt
```

## 启动

先启动本仓库机器人端（TCP `8080`），再：

```bash
cd /home/kaanh/Desktop/kaanhbin_wjy/client
python3 run_bridge.py \
  --policy-host 192.168.110.119 --policy-port 8000 \
  --robot-host 127.0.0.1 --robot-port 8080 \
  --mode async --camera-mode threaded \
  --hz 10 \
  --start-delay 1.0
```

公司实机已确认默认相机映射：**顶视 `video2`（`--top-camera` 默认 `2`）**，**右腕 `video0`（`--right-camera` 默认 `0`）**。此前反向映射会让相同模型表现明显变化。
右腕默认旋转 180°（`--right-rotate-180`，可用 `--no-right-rotate-180` 关闭）；顶视默认不旋转（可用 `--top-rotate-180` 开启）。

若没有顶视相机，传 `--top-camera=""`，不要用右腕画面冒充俯视。默认任务提示是 `pick and place`。  
返回的 `actions` 已是绝对值（前 6 维关节、第 7 维夹爪），客户端不再加一次当前状态。

默认启动后等待 `1` 秒再首次推理。`--post-idle-settle`（默认 `0.05s`）在 `remaining==0` 后再短暂等待，避免读到 batch 期间冻结的旧状态。

## 摄像头端口识别（抓图）

```bash
cd /home/kaanh/Desktop/kaanhbin_wjy/client
python3 capture_cameras.py
python3 capture_cameras.py --sources 0 2
python3 capture_cameras.py --sources /dev/video0 /dev/video2
```

输出例如：`camera_snaps/video0.jpg`、`camera_snaps/video2.jpg`。  
抓图默认仅对端口 `0` 做 180° 旋转（公司实机右腕相机倒装）。关闭：`--rotate-180-sources`（后不接端口）。

## 说明

- 默认 `--action-dim=7`；策略 batch 每行 7 个数（与 `send_follower_state` / 电机映射一致）。
- 机器人 TCP 默认端口 **8080**（与 `main.cpp` 一致）。
- 异步模式下，batch 执行期间也会刷新 `robot_q_`，推理可以读到当前关节。Ctrl+C 会发 `STOP_POLICY`。
