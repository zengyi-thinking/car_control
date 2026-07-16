# Car Control Decisions

## 2026-07-13 Beacon Continuation And Near Press

- 用四状态跟随流程替代分散的中心短冲、丢灯保持和附加衰减计时。
- `4000ms` 定义为从最后一帧有效边缘信标到速度归零的总窗口，A4 的 `280ms` 新鲜度等待包含在该窗口内。
- 边缘续航使用最后有效的归一化 `V -> beacon` 方向，不依赖 A4 保持新鲜，也不改变无人机通信协议。
- 近灯压灭与边缘续航互斥。进入近灯状态后清除边缘续航计时，避免压灭信标后继续前冲。
- 新信标接管时重置像素方向滤波起点，但保留速度矢量斜坡，兼顾第一帧响应和机械平顺性。
- `WAIT` 是安全默认状态。没有已接受的边缘方向时，小车不会因续航逻辑自行移动。
- 近灯压灯不再全程锁死旧方向：仅在新视觉帧、误差不小于 `2.5px` 且目标仍位于初始压灯方向前半平面时，以 `0.40` 系数渐进修正方向；保留 `0.64m/s` 压灯速度和原有计时上限。

## 2026-07-13 Remove C5 Car Motion Feedback

- 删除 `C5 cmd/measured velocity` 帧、状态标志、发送序号、发送 API 和 `app_car` 周期发送调用，小车不再把命令速度或编码器速度反馈给无人机。
- A4 18 字节接收格式、UART 配置、四状态信标跟随、近灯压灭、轮速 PID、手动控制和安全停车保持不变。
- 与飞控最简串级追踪环配套使用，飞控和小车必须同时重新烧录；小车双核构建为 `0 errors / 0 warnings`。

## 2026-07-15 Timed Landing Stop And Press Lock

- Reuse A4 flag bit 5 for the flight controller's 70-second `MISSION_STOP`; keep the frame at 18 bytes. The stop is latched until reboot so a later stale or ordinary tracking frame cannot restart the chassis.
- Wheel tuning and fresh manual joystick override remain available. When manual input is released, the mission stop latch regains control and returns the chassis to `SAFE_STOP`.
- Once `NEAR_PRESS` starts, a different candidate at `>=12px` is treated as loss of the current near lamp. The car continues only the existing bounded press direction/timer, then enters `WAIT_NEXT_EDGE` before accepting a new target.
- Entering `SAFE_STOP` uses the existing mode-transition reset to clear continuation, press, direction history, command ramps and PID output. No new uplink acknowledgement or protocol is introduced.

## 2026-07-16 Mission Stop Means Zero Wheel Output

- `SAFE_STOP` no longer runs yaw-hold correction. It preserves the heading target internally but commands zero translation, zero rotation and disables all four wheel outputs.
- Explicit manual override and wheel tuning keep their existing higher priority; releasing either returns the latched mission stop to a hard zero-output state.
