# Car Control Continuity

## Current Goal

小车在无人机边缘信标数据中断时沿最后有效方向短时续航，同时把 V 型车灯附近的信标作为压灭目标处理，避免压灯完成后继续盲走。

## Confirmed Behavior

- 小车到无人机的 C5 命令速度/编码器速度回传已彻底删除；A4 接收、`EDGE/HOLD/PRESS/WAIT`、里程计和轮速控制均保持原样。飞控和小车双核构建均为 `0 errors / 0 warnings`，本版需要两端同时重新烧录。
- 信标跟随状态为 `EDGE / HOLD / PRESS / WAIT`。
- 边缘信标最后一帧后最多续航 `4000ms`，速度基准为追踪速度的 `60%`，最后 `350ms` 在总窗口内衰减至零。
- 新的有效信标第一帧接管，方向修正继续受 `2.0m/s2` 矢量加速度限制。
- 近灯进入阈值为 `8px`，退出阈值为 `12px`。
- 近灯可见时持续压灯；丢失后再压 `250ms`；单次压灯总时长不超过 `1200ms`。
- 压灯期间对每个新视觉帧做受限方向修正；小于 `2.5px` 或信标进入初始压灯方向侧后方后锁定方向，避免近零抖动和掉头追灯。
- 压灯完成进入 `WAIT`，清除历史方向，只接受新的 `12px` 及以上边缘信标。
- 手动、轮组调试、安全停车和离开无人机追踪模式都会清除信标跟随状态。

## Interfaces

- A4/C5 帧格式未修改。
- `chassis_status_t` 增加跟随状态、剩余时间和历史方向有效标志，仅供小车本地诊断与 LCD 显示。
- 保留 `chassis_control_drone_lost_hold_ready()` 兼容入口，新代码使用 `chassis_control_drone_follow_ready()`。

## Verification

- 2026-07-13: `iar/build_all_cores.bat --no-pause` 双核构建通过，CM7_0/CM7_1 均为 `0 errors / 0 warnings`。
- 待实车验证边缘丢灯、A4 整帧丢失、近灯压灭以及手动接管路径。
