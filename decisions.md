# Car Control Decisions

## 2026-08-22 Restore The July 16 Press Speed Without Its Acceleration Step

- Restore `HW_DRONE_NEAR_PRESS_SPEED_MPS` to 0.64m/s, matching `car_control@ebcea5c` from July 16.
- Keep the current independent press acceleration at 0.85m/s2 instead of restoring the July controller's shared 2.0m/s2 acceleration. This recovers press authority without recreating a sharp tether load step.
- Preserve the current continuous visual lease, direction lock, wheel PID, yaw hold and A4/C5 formats.

## 2026-08-22 Restore Fast Continuous Beacon Response Without Reverting The State Machine

- Keep the current saved, slew-shaped command during short beacon gaps. Do not restore the July monolithic controller or recompute maximum speed from a stale direction.
- Raise far/minimum speed to 1.20/0.55 m/s, forward/right scale to 0.60/1.05, forward/right acceleration to 0.85/1.40 m/s2, filter alpha to 0.50, and press speed/acceleration to 0.52/0.85 m/s2. The effective pure-forward far cap is about 0.72 m/s.
- Extend only the fresh-A4 visual lease to 2000ms with a 500ms final fade. A true A4 link loss still stops at 280ms; mission stop remains permanently latched until power-cycle.
- Preserve wheel PID, yaw hold, coordinate signs, A4/C5 layouts and the positive-forward C5 lead weight already present in the user's worktree. This parameter round builds both car cores without warnings; real vehicle validation is pending.

## 2026-08-20 Lower The Forward Envelope One More Step

- Keep far, near and press speeds at 1.10, 0.48 and 0.40 m/s. Lower only the forward scale from 0.52 to 0.48, limiting pure forward or reverse motion to about 0.53 m/s.
- Lower forward-axis acceleration from 0.55 to 0.48 m/s2. Keep press and lateral acceleration, direction filtering, the one-second visual lease, yaw hold and wheel PID unchanged.
- Raise the positive-forward C5 command blend from 0.70 to 0.78 so the paired aircraft sees the gentler acceleration early. C5 stays odometry-led and remains measured during braking, reverse, lateral and manual motion.
- This is an uncommitted paired-flight parameter set. Physical validation is pending.

## 2026-08-20 Reserve Forward Chase Margin For The Drone

- Keep the current continuous-motion lease, direction filter, near-press behavior, yaw hold, wheel PID and A4/C5 protocols unchanged.
- Reduce only the automatic forward scale from 0.65 to 0.58 and the forward slew limit from 1.0 to 0.70 m/s2. With the 1.20 m/s base speed, the effective forward cap is about 0.70 m/s. Lateral and press parameters do not change.
- This paired limit gives the drone time to establish feed-forward tilt before tether load and visual error accumulate. Both cores build without errors or warnings; vehicle validation is pending and the change remains uncommitted.

## 2026-08-20 Restore One Step Of Competition Response Without Full Aggression

- Keep the existing 1000 ms visual-loss lease, saved slew-limited command, direction interpolation, wheel PID, yaw hold and A4 format.
- Raise automatic far/minimum/press speed to 1.20/0.52/0.45 m/s, forward/right scale to 0.65/1.00, forward/right acceleration to 1.0/1.2 m/s2 and press acceleration to 0.80 m/s2.
- Do not restore the old 2.0 m/s2 acceleration in this round. The moderate values shorten visible startup delay while retaining margin for the drone to follow. Dual-core IAR build passes; physical validation is pending.

## 2026-08-12 Restore Direct Remote Control And Drone Priority

- Restore the original remote-mode entry rule: any fresh, checksum-valid A3 frame selects direct manual control when no higher-priority drone action is active. Remove the later raw-axis override threshold.
- Use only two external motion sources. Valid A4 tracking and its bounded continuation outrank A3 remote control; mission/local timed stop outranks both. Idle heading hold is not an external motion source.
- Use only the left stick for motion: axis 0 commands lateral speed and axis 1 commands forward speed. Axes 2/3 are ignored and heading always holds the startup direction.
- Keep the legacy position-edit implementation isolated from every normal mode so it cannot become a third motion source or reinterpret the right stick. Keep the 18-byte A3 layout for transmitter compatibility.
- Add receive diagnostics to the LCD so no bytes, invalid frames, stale frames and fresh frames can be distinguished without a debugger.

## 2026-08-12 Restore Idle Heading Hold And Slow Beacon Tracking

- Add a distinct `HEADING_HOLD` mode for ordinary idle operation. It commands zero translation and uses the existing IMU heading controller to restore the startup heading.
- Keep mission stop semantics unchanged: local 60-second stop and A4 `MISSION_STOP` still select hard-zero `SAFE_STOP`, so landing cannot cause the car to rotate.
- Reduce automatic beacon tracking to `1.00m/s` far speed, `0.45m/s` near speed, `0.50m/s` press speed, `0.90` lateral scale and `1.2m/s2` vector acceleration. Begin interpolation at `60px` and leave wheel PID and manual speed unchanged.

## 2026-07-17 Restore Local 60-Second Stop

- Count elapsed time from local car-control startup using the existing `5ms` control period.
- At `60000ms`, set the existing mission-stop latch and reuse the hard-zero `SAFE_STOP` path; no new protocol or control state is added.
- Keep wheel tuning and fresh manual override above the latched stop, matching the existing A4 mission-stop behavior.

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

## 2026-08-17 Short Vision-Loss Lease For Continuous Tracking

- In `DRONE_TRACK`, a missing `MOVE_VALID` frame no longer immediately selects manual or heading hold. The application keeps a `1000ms` tracking lease so the chassis can consume its last reliable V-to-B direction without resetting the mode or direction filter during a longer visual dropout.
- During that lease, the chassis keeps the previous translation command for about `700ms`, then fades it to zero over `300ms`. A4 format, coordinate signs, wheel PID, and near-press logic are unchanged. The independent A4 link timeout remains `280ms`; a missing link is not treated as a visual dropout.
- The lease is deliberately bounded; after it expires, the car returns to manual control when a fresh A3 exists, otherwise to heading hold. This prevents the previous multi-second blind continuation from hiding a real link or visual failure.

## 2026-08-17 Smooth Automatic Approach And Press

- Reduce the automatic far target speed from `1.30m/s` to `1.10m/s`, the near minimum from `0.60m/s` to `0.45m/s`, and the V-to-B direction filter new-sample weight from `0.65` to `0.35`.
- Limit ordinary tracking velocity-vector acceleration to `1.0m/s2`. Keep this shaping above the existing wheel-speed PID so wheel calibration and yaw hold remain unchanged.
- Give `NEAR_PRESS` its own `0.60m/s2` velocity-vector acceleration limit and reduce press speed from `0.50m/s` to `0.35m/s`. The press timer, direction lock, B1/B2 arbitration, and A4 protocol remain unchanged.
- 2026-08-12：运行时运动来源收口为无人机A4和LoRa遥控A3。安全停车优先，其次是有效A4追踪及有界续航，再其次是新鲜A3遥控；左摇杆同时控制前后和左右，右摇杆与旧位移提交链路不再参与运动仲裁。普通待机使用 `HEADING_HOLD` 回到上电锁定方向。
## 2026-08-20 Lead C5 Only During Positive Forward Acceleration

- Preserve the nine-byte C5 protocol and keep measured odometry as the normal reference.
- In automatic tracking only, when the slew-shaped positive-forward command is above measured speed, report measured speed plus 55 percent of that gap. Do not prefeed braking, reverse, lateral or manual motion.
- This is an upstream tracking aid for the aircraft, not a change to chassis velocity, acceleration, wheel PID, beacon continuation or press behavior.
- Both cores build without errors or warnings. Vehicle and tethered-flight validation are pending, so the change remains uncommitted.
