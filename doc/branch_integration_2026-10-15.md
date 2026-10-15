# kaanh_test 实机主线整合（2026-10-15）

整合结果保存在本地 `kaanh_test`，用于后续实机验证；不推送远端，不删除旧分支或 stash。
本地 `kaanh` 恢复到合并前的 `59ef1fd`，验证完成后再决定合入。
整合前本地 kaanh：`59ef1fd`；远端 kaanh：`1470e10`。
备份分支：`backup/kaanh-before-integration-20261015`。

## 功能来源

| 来源 | 整合处理 |
| --- | --- |
| `kaanh_rtc` / `switch` `97feb31` | 同一提交，合并一次；保留异步执行、轨迹衔接、融合及 10 月 9 日实机默认值。 |
| `kaanh_vr_op` `c512061` | 已是 RTC 祖先；VR 锚点、reset、关节保护、85mm 夹爪和实机 VLA 部署已包含。 |
| `kaanh_vr_op_log` `dd6b648` | 合入日志会话、动态密码工具、启动自动登录与测试；日志实现适配 RTC 多线程 bridge，不恢复旧同步执行逻辑。 |
| 本地与远端 `kaanh` | 保留两侧历史。保留源目录 XML 查找及完整参数处理；空 XML 问题采用 RTC 的运行目录同步和不再调用启动 saveCs 的修复。 |
| `master` / `master_dev` | 移植 `fb4e429` 的 m_init 未使用臂保持反馈位置行为，保留当前单臂支持和实机 preset；reset 已被新 VR 实现覆盖，不整分支合并。 |
| `test_policy` / `dev/async-inference` | 核对无夹爪误读其他关节、batch skip 和中途替换修复；RTC 已有对应新实现，不恢复旧版阈值、状态冻结或整分支覆盖。 |
| `dev/async-ttc` | 按本轮范围暂缓多候选采样、medoid/critic 和自适应预算。 |
| `dev/vr_dev` | 按本轮范围暂缓 GET_STATE / GET_EE_STATE / SET_EE_TARGET 旧接口和对应旧控制路径。 |

## 合并后的行为

- RTC 默认参数保持不变：手臂 0.3rad/s、1.5rad/s²，future_handoff=1、jerk_smooth=0；夹爪 50mm/s。async 和指数融合默认开启。
- `policy --log=1` 默认启用会话日志，`policy --log=0` 关闭。bridge 常驻时，每次 policy 启动建立独立目录，停止或状态查询失败后关闭日志。
- 保留 RTC 的 events.jsonl、bridge.log、观测图像及分析脚本。状态增加 log_enabled 和字符串 log_session，现有协议字段保留。
- 日志会话操作在 Python 中加锁；跨会话推理结果不会写入新会话。C++ 日志标记使用原日志分支的无锁原子状态。
- 自动登录在 cs.init() 后尝试；登录失败保留诊断及人工登录路径，不输出密码。
- 三份已跟踪运行日志仅移出 Git 索引，本地文件保留。日志目录和 .worktrees 忽略。
- stash 0 只有日志；stash 1 的相关文件内容与 4a02c51 相同，不重复应用，两份 stash 均保留。

## 离线验证

使用本机 client 环境执行：

```bash
/home/kaanh/miniconda3/envs/client/bin/python -m unittest discover -s tests -p 'test_*.py'
cmake -S . -B build/integration -DA10_BUILD_STARTUP_LOGIN_TEST=ON
cmake --build build/integration -j 4
ctest --test-dir build/integration --output-on-failure
```

Python 40 项通过。C++ 9 个离线测试程序通过：锚点/reset 协议、batch、handoff、三次/五次轨迹、平滑器、关节诊断、关节保护、日志生命周期。另有 SDK 登录测试通过，完整控制器构建通过。
离线 C++ 测试使用 g++ -std=c++17 -O2 -pthread，包含项目根目录和 vendor/rtb-1.0.0/include；锚点测试额外链接 a10_anchor_protocol.cpp。

整合过程未启动控制器或发送运动指令。用户已确认免密登录实机验证通过；其余整合功能待用户后续验证，m_init 双臂保持行为及任务效果仍需现场核验。
