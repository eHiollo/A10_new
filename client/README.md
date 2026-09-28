# client（VLA / OpenPI 桥接）

动态口令工具见 [KAANH_PASSPORT.md](KAANH_PASSPORT.md)。

把 OpenPI 推理服务（websocket）与本仓库 `a10_tcp_server.cpp` 的机器人 TCP 服务桥接起来。  
原独立目录：`/home/kaanh/Desktop/kannhbin_client`，已并入本仓库 `client/`。

## 已实现能力

- 对接 OpenPI websocket 推理服务（`msgpack + numpy` 协议）
- 对接本仓库 `a10_tcp_server.cpp` 行分隔 TCP 协议
  - 请求机器人状态：`GET_FOLLOWER_STATE`
  - 单行即时目标（与策略 batch 无关）：`{"q":[...]}\n` → `target_q_`
  - **策略轨迹（多步）**：`SET_JOINTS_BATCH {"actions":[[7 floats],...]}\n` → 服务端入队 `target_q_batch_` 并**立即回一行** `{"accepted":true,"queued":T}\n`（失败为 `accepted:false`）
  - 策略是否执行完：`GET_POLICY_STATUS` → `{"idle":bool,"remaining":int}`（`remaining` 为队列中尚未执行的步数）
- 从 USB 摄像头读取图像，按推理服务约定原样发送（不做归一化、不在客户端缩放到 224）
  - `observation/state`: `float32 (7,)`，关节绝对角 + 夹爪绝对开度（mm，约 0–85）
  - `observation/images/right`: `uint8 HWC` RGB，原始分辨率
  - `observation/images/top`: `uint8 HWC` RGB，原始分辨率；不接顶视相机时不发这个键
- 组包观测并发起推理；将整段 `actions` 经 **`SET_JOINTS_BATCH`** 下发后，**轮询 `GET_POLICY_STATUS` 至 `remaining==0`**，再进入下一轮
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
  --hz 10 \
  --start-delay 1.0
```

默认相机映射：**顶视 `video0`（`--top-camera` 默认 `0`）**，**右腕 `video2`（`--right-camera` 默认 `2`）**。  
右腕默认旋转 180°（`--right-rotate-180`，可用 `--no-right-rotate-180` 关闭）；顶视默认不旋转（可用 `--top-rotate-180` 开启）。

若没有顶视相机，传 `--top-camera=""`，不要用右腕画面冒充俯视。默认任务提示是 `pick and place`。  
返回的 `actions` 已是绝对值（前 6 维关节、第 7 维夹爪），客户端不再加一次当前状态。

默认启动后等待 `1` 秒再首次推理。`--post-idle-settle`（默认 `0.05s`）在 `remaining==0` 后再短暂等待，避免读到 batch 期间冻结的旧状态。

## 实机运行日志与耗时定位

日志开关由**机器人终端的 policy 命令**控制：

```text
policy --log=1    # 开启本次 policy 的日志
policy --log=0    # 关闭本次 policy 的日志
policy           # 默认 log=1
stop             # 停止 policy，桥接程序读到停止状态后关闭本次日志
```

先重新编译并重启机器人程序，使新增参数和状态接口生效：

```bash
cmake --build build --target kaanhbin -j2
```

构建会同步根目录 `kaanh.xml` 到 `build/bin/kaanh.xml`。如果启动机器人时指定了其他 XML，需要在该文件的 `A10PolicyTcpDriver/Command/GroupParam` 下增加 `<Param name="log" abbreviation="l" default="1"/>`。

Python 桥接程序正常启动即可，不再接受 `--log` / `--no-log`；只保留 `--log-dir` 用于指定保存位置：

```bash
cd /home/kaanh/A10_new/client
python3 run_bridge.py --policy-host 192.168.110.119 --robot-host 127.0.0.1
# 可选：在原有启动参数上增加 --log-dir /指定的日志根目录
```

每次 policy 实际开始运行，机器人都会生成新的日志会话编号。桥接程序在每轮推理前和等待动作执行时读取 `GET_POLICY_STATUS`，依据其中的 `policy_running`、`log_enabled`、`log_session` 创建或关闭日志。因此即使 Python 一直运行，`stop` 后再次 `policy --log=1` 也会新建一组文件；`policy --log=0` 不创建日志目录。桥接程序自身重启后，也会为当前启用日志的 policy 新建一组文件。

日志写在**运行 Python 桥接程序的电脑**上。默认根目录是启动命令所在目录下的 `logs/`；例如从 `/home/kaanh/A10_new/client` 启动，则每次会话目录为：

```text
/home/kaanh/A10_new/client/logs/run_20260928T060000_123456Z_pid12345_ab12cd34/
  vla_bridge.log
  vla_timing.jsonl
```

目录名包含桥接程序开始记录时的 UTC 时间、进程号和唯一后缀；终端会打印实际绝对路径和对应的机器人会话编号。

- `vla_bridge.log`：连接、告警、异常堆栈及每 20 轮动作摘要。
- `vla_timing.jsonl`：每轮阶段耗时，以及 `policy_session_start` / `policy_session_end` 会话事件，其中包含机器人会话编号。`cycle_start` 和 `stage_start` 可定位尚未返回的调用，`cycle_end` 记录结果和失败阶段。

开关在桥接程序下一次收到状态回包时生效；若正在阻塞等待相机或推理结果，需等该调用返回后才能同步。执行期间发生 stop 或会话切换时，旧轮次剩余的耗时不会写入新会话。桥接程序启动但尚未观察到启用日志的 policy 时，仅保留终端输出；旧版本机器人缺少这些状态字段时，会提示升级，并保持文件日志关闭。

cycle_end 中，state_ms 是机器人状态请求耗时；camera_ms 是相机取帧耗时；inference_ms 是发送观测至收到并解析推理结果的客户端耗时；server_infer_ms 是服务端返回的内部推理耗时（若有）；send_batch_ms 包含动作序列化和机器人确认；execute_wait_ms 包含轮询和实机执行等待；settle_ms 是配置的状态刷新等待。cycle_ms 是本轮总耗时，不含按 --hz 补足的节流时间。status_polls 是状态查询次数，image_bytes 是原始图像字节数。异常轮次记录 outcome=error、failed_stage 和 error_type；详细堆栈见普通日志。

每次运行的两个文件分别按 20 MiB 轮转，各保留 5 份轮转文件；不同运行目录互不覆盖，旧运行目录不会自动删除。JSON 时间戳和目录名使用 UTC，普通文本日志使用本机时间，耗时使用单调时钟。日志不保存图像或整段动作。机器人进程原有的控制台输出需在启动机器人时另行重定向保存，例如 ./build/bin/kaanhbin > robot.log 2>&1；本功能不会改变机器人实时控制线程的写盘路径。

## 摄像头端口识别（抓图）

```bash
cd /home/kaanh/Desktop/kaanhbin_wjy/client
python3 capture_cameras.py
python3 capture_cameras.py --sources 0 2
python3 capture_cameras.py --sources /dev/video0 /dev/video2
```

输出例如：`camera_snaps/video0.jpg`、`camera_snaps/video2.jpg`。  
抓图默认仅对端口 `0` 做 180° 旋转（与顶视倒装一致）。关闭：`--rotate-180-sources`（后不接端口）。

## 说明

- 默认 `--action-dim=7`；策略 batch 每行 7 个数（与 `send_follower_state` / 电机映射一致）。
- 机器人 TCP 默认端口 **8080**（与 `main.cpp` 一致）。
- batch 执行期间机器人端会暂停刷新 `robot_q_`；请在 `GET_POLICY_STATUS` 为 `remaining==0` 后再读状态做观测/推理。
