# 控制器动态口令

`kaanh_passport.py` 按厂商 SDK 的动态口令规则，在本机推导 `Manufacturer` 和
`Developer` 角色的口令：对十进制字符串 `年 + 月 + 日` 求 MD5；`Developer`
再加当前小时。日期和小时以控制器本地时间为准。控制器时钟或时区与运行工具的
机器不一致时，请显式传入控制器上的日期和小时。

```bash
python3 client/kaanh_passport.py --command
python3 client/kaanh_passport.py --user Developer --date 2026-05-26 --hour 8 --command
```

`--command` 输出用于控制器命令接口的 `login` 指令；不带此选项只输出口令。
该命令接口与 `8080` 动作端口不同。工具只在本地计算，不会连接机器人。
输出是有效凭据，请勿写入仓库或运行日志。
