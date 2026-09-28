# client（VLA / OpenPI 桥接）

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
