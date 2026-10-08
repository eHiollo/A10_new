# client（VLA / OpenPI 桥接）

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
