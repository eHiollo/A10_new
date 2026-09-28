# 控制器动态口令

## 启动时自动登录

重新编译本分支的 `kaanhbin` 后，启动程序会在 `cs.init()` 完成后，使用控制器
进程的本地日期生成厂家口令，直接调用 SDK 的 `UserManagement::login()`。
检查当前角色确实为 `Manufacturer` 后，终端输出：

```text
自动登录成功：当前用户为 Manufacturer（厂家）。
```

自动登录失败会输出中文失败提示，并保留手动登录入口。如果 SDK 已设置登录角色，
但后续写审计日志异常，会保留成功提示并追加日志配置告警。实现不会打印口令或
SDK 异常原文；自动登录本身不会执行上电、使能或运动命令。Linux 的 `sudo`
密码验证仍按系统配置执行。

在本分支工作目录中编译并启动（执行文件位于 `build/bin/`）：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target kaanhbin -j2
cd build/bin
sudo ./kaanhbin
```

验证自动登录时可以只运行 SDK 认证测试，不启动机器人：

```bash
cmake -S . -B build -DA10_BUILD_STARTUP_LOGIN_TEST=ON
cmake --build build --target a10_startup_login_test -j2
ctest --test-dir build -R '^a10_startup_login$' --output-on-failure
```

## 本地口令工具

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
