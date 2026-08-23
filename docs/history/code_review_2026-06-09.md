# 历史代码审查（非当前事实来源）

本报告生成于2026-06-09，所引用的引脚、符号和行号已经随之后的底盘调整发生变化。保留它仅用于追溯，不应据此修改当前轮序或坐标；当前事实以根目录README、`hw_board_pins.h`和 `hw_config.h` 为准。

---
phase: manual-control-convergence-review
reviewed: 2026-06-09T09:27:50Z
depth: standard
files_reviewed: 12
files_reviewed_list:
  - code/app/app_car.c
  - code/control/chassis_control.c
  - code/control/chassis_control.h
  - code/control/mecanum.c
  - code/control/mecanum.h
  - code/control/odometry.c
  - code/hardware/hw_board_pins.h
  - code/hardware/hw_display.c
  - code/hardware/hw_encoder.c
  - code/hardware/hw_motor_drv8701e.c
  - code/protocol/proto_remote.c
  - code/protocol/proto_remote.h
findings:
  critical: 0
  warning: 2
  info: 0
  total: 2
status: issues_found
---

# Phase manual-control-convergence-review: Code Review Report

**Reviewed:** 2026-06-09T09:27:50Z
**Depth:** standard
**Files Reviewed:** 12
**Status:** issues_found

## Summary

Reviewed the current manual-control convergence path end to end: board-level wheel mapping, encoder/motor sign handling, joystick X/Y conversion, `mecanum_inverse` cardinal-direction behavior, stick-release zeroing, enabled gating, and display semantics.

Two defects remain in scope:

1. The current motor pin remap is inconsistent with the encoder remap and with the wheel-map comments immediately above it, so wheel 0 and wheel 2 appear cross-wired at the motor layer.
2. Manual joystick X is displayed/stored in one user-space sign convention, then converted into internal `vy` with the opposite sign, so lateral manual motion is inverted relative to the advertised X axis.

`mecanum_inverse` itself is algebraically correct for forward/back/left/right under the documented body-frame convention, and the stick-release path does correctly zero commands, reset PID state, and drop `enabled`.

## Warnings

### WR-01: Motor Remap Does Not Match Encoder Remap For LF And RR

**File:** `code/hardware/hw_board_pins.h:19-42`
**Issue:** The file comments define the internal wheel order as `0=LF, 1=RF, 2=RR, 3=LR`, then state the physical remap should be `LF->motor3`, `RF->motor4`, `RR->motor2`, `LR->motor1` and `LF->encoder3`, `RF->encoder4`, `RR->encoder2`, `LR->encoder1`. The encoder macros follow that remap: wheel 0 uses `P07.6/P07.7` (legacy encoder3) and wheel 2 uses `P19.2/P19.3` (legacy encoder2). The motor macros do not: wheel 0 uses `P05.2/P05.3` (legacy motor2) while wheel 2 uses `P09.0/P09.1` (legacy motor3). Based on the repository’s prior board map in `old_control/legacy_car_control/docs/car_control_detail.md:53-64`, this swaps LF and RR at the motor layer while the encoder layer keeps them distinct. The result is that PID/odometry label wheel 0 as LF and wheel 2 as RR, but actuation appears to send wheel 0 output to RR hardware and wheel 2 output to LF hardware.
**Fix:**
```c
/* Make motor remap match the documented wheel remap, after bench verification. */
#define HW_MOTOR0_DIR_PIN  P09_1
#define HW_MOTOR0_PWM_PIN  TCPWM_CH24_P09_0   /* LF -> motor3 */

#define HW_MOTOR2_DIR_PIN  P05_3
#define HW_MOTOR2_PWM_PIN  TCPWM_CH11_P05_2   /* RR -> motor2 */
```
If the hardware was rewired differently, then update the wheel-order comments, display labels, and reference docs together so motor and encoder ownership stay consistent for each internal wheel index.

### WR-02: Manual X Axis Is Inverted When Converted Into Internal `vy`

**File:** `code/control/chassis_control.c:450-459`
**Issue:** `joy_x` is explicitly negated from the raw remote channel, then stored and displayed as `joy_x_norm` / `user_cmd_x_mps`. The project’s current body-frame docs say `+x` is left and internal `vy > 0` is also left (`code/hardware/hw_board_pins.h:90-97`, `code/control/mecanum.h:10-15`). But the manual branch converts `user_cmd_x_mps` with `vy_body_cmd = -g_status.user_cmd_x_mps`. That means a positive displayed/user X command produces a negative internal `vy`, so left/right manual motion is reversed relative to the advertised X axis. Because the display shows `JOYX` and `X` before that conversion (`code/hardware/hw_display.c:196-197`), the screen can claim positive X while the wheel-duty pattern is the opposite lateral move.
**Fix:**
```c
/* If user X is sketch +x / left, keep the same sign into internal vy. */
vx_body_cmd = g_status.user_cmd_y_mps;
vy_body_cmd = g_status.user_cmd_x_mps;
```
If the intended user convention is actually `X > 0 == right`, then remove the initial negation of `joy_x` and rename the display/docs accordingly instead of carrying conflicting sign conventions.

---

_Reviewed: 2026-06-09T09:27:50Z_
_Reviewer: Claude (gsd-code-reviewer)_
_Depth: standard_
