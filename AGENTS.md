# car_control Agent Guide

本文件保存小车工程的长期维护规则。当前运行状态见 `CONTINUITY.md`，长期取舍见 `decisions.md`；历史参数以Git记录为准，不应重新写回当前文档。

## 工程边界

- 平台：Infineon CYT4BB7双核工程，C语言和IAR Embedded Workbench。
- `code/app/`：模式仲裁、协议新鲜度、停车锁存和最终电机启停。
- `code/control/`：麦轮运动学、编码器里程计、轮速PID、航向保持和信标接近/压灯。
- `code/hardware/`：电机、编码器、IMU、显示和板级配置。
- `code/protocol/`：A3遥控、A4视觉与C5速度反馈。
- `user/`：双核入口和中断，不放业务状态机。

## 当前稳定接口

- A3：18字节遥控帧。
- A4：18字节飞控视觉/停车帧。
- C5：9字节小车速度反馈。
- 小车车体坐标固定为 `+X`向右、`+Y`向前。
- 历史字段 `vx_body`表示前向、`vy_body`表示右向；维护协议时优先写明方向语义，不根据字段字母猜坐标。
- 轮序固定为LF、RF、LB、RB；电机和编码器符号只在板级适配边界修正。

协议字段、坐标、帧长、校验或标志位改变时，必须同步检查 `flight_control` 的生产者/消费者并配套烧录。

## 控制与安全

控制优先级：

1. 已锁存的A4 `MISSION_STOP`。
2. 新鲜有效A4及其有界视觉丢帧租约。
3. 新鲜有效A3遥控。
4. 无运动命令时的零平移航向保持。

- `MISSION_STOP`进入 `SAFE_STOP` 后四轮硬零，只能重新上电解除。
- 小车没有本地任务倒计时；停车时刻由飞控统一决定。
- A4真实链路超时与“链路在线但信标短失”是不同状态，禁止用长视觉租约掩盖通信断线。
- `SAFE_STOP`中不得用航向回正重新驱动电机。
- 改速度、加速度、压灯或丢帧保持时，一轮只改一个主要机制，并先架空四轮验证方向和停车。

## 工作流程

1. 读本文件和 `CONTINUITY.md`。
2. 检查分支、工作区、最近提交及实际构建脚本。
3. 从 `code/hardware/hw_config.h`读取当前参数，不从旧决策抄值。
4. 只检索 `decisions.md` 中与任务相关的当前决策。
5. 协议问题同时读取飞控的 `docs/car_link_protocol.md` 和 `docs/coordinate_systems.md`。
6. 保留用户未提交资产；跨仓修改使用各自独立提交。

## 构建与验证

```bat
iar\build_all_cores.bat --no-pause
```

控制、协议、硬件、API或IAR工程改变后必须构建两个核心，达到 `0 errors / 0 warnings`。纯文档修改只需检查 `git diff --check`、链接和过期表述。

验证等级必须区分：

```text
implemented -> built -> wheels-off bench -> ground test -> paired task -> accepted
```

构建成功不能证明方向、连续性、压灯或锁存停车已经通过实车验证。

## 文档规则

- `README.md`面向使用者。
- `CONTINUITY.md`只保存当前参数语义、验证结果、未决问题和恢复步骤。
- `decisions.md`只保存当前长期决策与少量被实测否定的方案。
- 日期化审查放入 `docs/history/`。
- 不在长期文档堆叠多代速度、临时猜测和完整构建日志。
