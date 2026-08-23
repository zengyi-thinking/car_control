# car_control

基于 Infineon CYT4BB7 的四轮麦克纳姆小车控制工程。小车可以由LoRa遥控器直接控制，也可以接收 `flight_control` 发布的V灯/信标坐标，沿V→B方向追踪并压灭信标。

> 安全提示：首次烧录时应架空四轮验证方向。轮序、编码器符号或电机接线错误会导致底盘突然横移、旋转或失控。

## 当前能力

- 200Hz底盘控制节拍。
- 四轮编码器里程计、速度PID与静摩擦前馈。
- 上电航向锁定和运动中Yaw保持。
- A3遥控手动模式。
- A4无人机视觉追踪、短时丢帧连续运动和近灯压灭。
- C5车体速度反馈，帮助无人机建立追车前馈。
- `MISSION_STOP`永久锁停：收到一次合法停车标志后四轮硬零，重新上电前遥控和普通A4均不能恢复。

## 坐标与轮序

小车公开车体坐标采用：

| 轴 | 正方向 |
|---|---|
| `+X` / `right_x` | 小车右方 |
| `+Y` / `forward_y` | 小车前方 |

内部速度字段中，`vx_body`表示前向速度，`vy_body`表示右向速度。这是历史命名，外部协议应优先使用 `forward_y/right_x` 语义。

全工程轮序固定为：

```text
0 = LF 左前
1 = RF 右前
2 = LB 左后
3 = RB 右后
```

电机、编码器引脚和单轮符号以 [`code/hardware/hw_board_pins.h`](code/hardware/hw_board_pins.h) 为准。不要只修改麦轮公式来补偿单轮接反。

## 工程结构

```text
code/app/          模式仲裁、协议新鲜度和电机输出
code/control/      麦轮运动学、里程计、轮速PID和信标任务
code/hardware/     电机、编码器、IMU、显示与板级引脚
code/protocol/     A3遥控、A4视觉和C5速度协议
user/              双核入口与中断
iar/               IAR工程、构建和烧录脚本
libraries/         CYT4BB7 SDK与逐飞外设库
```

## 控制优先级

从高到低：

1. 已锁存的 `MISSION_STOP`：进入 `SAFE_STOP`，四轮硬零。
2. 新鲜有效A4及其有界视觉丢帧租约：`DRONE_TRACK`。
3. 新鲜A3遥控：`MANUAL`。
4. 无外部运动命令：`HEADING_HOLD`，零平移并保持上电航向。

小车当前没有独立的本地定时停车；任务停车时间由飞控决定并通过A4 bit5下发。

## 通信

| 链路 | 串口 | 波特率 | 帧 |
|---|---|---:|---|
| LoRa遥控 | UART1，P04_1/P04_0 | 115200 | A3，18字节 |
| 无人机 | UART5，P02_1/P02_0 | 115200 | A4，18字节；C5，9字节 |

A4/C5完整格式见配套飞控仓库的 `docs/car_link_protocol.md`。线协议、坐标符号或标志位发生变化时，必须同时更新和烧录飞控与小车。

## 构建

需要Windows和IAR Embedded Workbench for Arm 9.2系列。如果IAR不在 `PATH` 中：

```bat
set IAR_EW_ROOT=C:\path\to\Embedded Workbench 9.2
```

构建双核：

```bat
iar\build_all_cores.bat --no-pause
```

构建并烧录：

```bat
iar\download_all_cores.bat --no-pause
```

全新克隆第一次烧录前，需在IAR中分别打开 `iar/project_config/cyt4bb7_cm_7_0.eww` 和 `cyt4bb7_cm_7_1.eww`，选择正确调试器并让IAR生成本机C-SPY设置；这些个人设置不会提交到仓库。

## 关键配置

运行参数集中在 [`code/hardware/hw_config.h`](code/hardware/hw_config.h)：

- `HW_DRONE_TRACK_SPEED_MPS`：远区基准速度。
- `HW_DRONE_TRACK_FWD_SCALE` / `RIGHT_SCALE`：前后与左右比例。
- `HW_DRONE_TRACK_FWD_ACCEL_MPS2`：前后加速度限制。
- `HW_DRONE_TRACK_LOST_GRACE_MS`：视觉短时丢失租约。
- `HW_DRONE_NEAR_PRESS_SPEED_MPS`：近灯压灭速度。
- `HW_DRONE_C5_FORWARD_ACCEL_LEAD_WEIGHT`：正向加速C5前馈比例。

修改参数后先架空验证起步、换向、停车和四轮方向，再进行地面任务测试。

## 验证重点

1. 架空四轮，分别命令前、后、左、右，确认轮序和编码器符号。
2. 静止约1秒后确认航向锁定完成；手动转动车体后应回到上电航向。
3. A4中的V→B向量右/前为正时，小车应分别向右/向前。
4. 短时失去信标时运动应连续，链路真正超时后必须停车。
5. `MISSION_STOP`置位后发送普通A4和满量遥控，小车仍必须保持四轮硬零。

## 许可证

本项目自研代码以 GNU General Public License v3.0 or later 发布，完整条款见 [`LICENSE`](LICENSE)。`libraries/` 中的第三方组件继续适用其各自的版权和许可证声明，详见 [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md)。
