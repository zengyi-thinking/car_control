# 航向锁定（Yaw Hold）全程生效设计

## 1. 背景

无人机遥控小车要求小车移动方向准确，前提是小车航向（yaw）在全程保持稳定：
上电后无论运动中、还是被手动转动，都要自动回正到锁定角度。

旧逻辑只在 MANUAL / DRONE_TRACK 两个模式调用 `chassis_yaw_hold`，
SAFE_STOP（无遥控、无无人机）模式下完全不修正航向，电机被 `hw_motor_drv8701e_stop_all` 关停。
导致手持抬起轮子转动小车时不回正。

## 2. 决策

航向锁定的调用与模式解耦：**只要 `yaw_lock_valid` 有效，所有模式（含 SAFE_STOP）都做纯旋转修正**，
平移指令在 SAFE_STOP 下强制为 0（不允许无指令自由漂移）。

## 3. 实现（全部在 chassis_control.c）

### 3.1 放宽上电启动锁
- `CHASSIS_STARTUP_LOCK_TIME_S`：`2.0f` → `1.0f`
- `CHASSIS_STARTUP_YAW_STABLE_DEG`：`4.0f` → `8.0f`
- `chassis_startup_lock_step` 内硬编码的 `>= 2.0f` 改为 `>= CHASSIS_STARTUP_LOCK_TIME_S`
- 效果：上电约 1 秒即锁定当前 yaw，容忍窗口内 ±8° 抖动；gyro bias 仍在该 1s 窗口内做平均。

### 3.2 SAFE_STOP 分支增加纯航向保持
在 `case CHASSIS_MODE_SAFE_STOP`（约 line 1469）末尾：
```c
if (g_runtime.yaw_lock_valid)
{
    vx_body_cmd = 0.0f;
    vy_body_cmd = 0.0f;
    omega_cmd = chassis_yaw_hold(yaw_deg, gyro_z_radps, dt_s);
    command_active = 1U;
    g_status.target_yaw_deg = g_runtime.yaw_lock_deg;
    g_status.heading_valid = 1U;
}
```

### 3.3 enabled 判定允许 SAFE_STOP 航向修正
`g_status.enabled`（约 line 1562）增加条件：
```c
((g_status.mode == CHASSIS_MODE_SAFE_STOP) &&
 g_runtime.yaw_lock_valid &&
 command_active)
```
使 app_car.c 的 `status->enabled` 判定为真，走 `hw_motor_drv8701e_set_all` 而非 stop_all。

## 4. 安全保护（已存在，未改动）

- `chassis_yaw_hold` 死区：误差 <0.4° 且 |gyro|<0.010rad/s 时返回 0，电机不动，无微震。
- `omega_cmd` 限幅 ±`HW_MAX_YAW_RATE_RADPS`（2.4 rad/s）。
- `mecanum_desaturate` + `HW_MANUAL_MAX_DUTY` 限幅，不过载。
- WHEEL_TUNE 模式优先级最高（app_car.c，`g_chassis_wheel_pid_tune_enabled` 仲裁在最前），与 SAFE_STOP 物理互斥，调参流程不受影响。

## 5. 不变的部分

- MANUAL / DRONE_TRACK / WHEEL_TUNE / POSITION_MOVE 各分支逻辑不变。
- 航向环 Kp/Ki/Kd（2.4/0.03/0.28）不变。
- 协议、遥测字段、跨核共享内存不变。

## 6. 验证

`iar/build_only.bat` 双核构建：CM7_0 / CM7_1 均 0 errors / 0 warnings。
