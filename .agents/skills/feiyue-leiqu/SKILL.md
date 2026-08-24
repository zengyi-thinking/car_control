---
name: feiyue-leiqu
description: 飞跃雷区三工程协同开发规范。用于 flight_control 飞控架构与任务扩展、car_control 麦克纳姆小车、camera_helper 副摄的设计、诊断、调参、视觉识别、A4/C5 与 B1/B2 通讯、跨核共享状态、硬件联调和经验沉淀；也适用于无人机追车、V灯、信标、光流、TOF、姿态、高度、Yaw、电机或轮速稳定性问题。不用于无关的通用 C 编程。
---

# 飞跃雷区 Skill

把飞控、小车和副摄视为一个有清晰责任边界的闭环系统。目标不是一次性调出一组参数，而是让每次开发都留下可复用、可验证、不会污染后续判断的工程能力。

## 开始任何任务

1. 识别当前仓库以及同级目录中是否存在另外两个配套仓库。不得因它们存在就自动扩大修改范围。
2. 完整读取当前仓库的 `AGENTS.md`，遵守其中的工具路由；有 CodeGraph 时优先用它回答结构关系。
3. 检查分支、工作区、最近提交和实际存在的构建脚本，保护用户已有修改。
4. 读取当前仓库 `README.md`。若存在 `CONTINUITY.md`，只读取当前状态、工作集和 open loops；在 `decisions.md` 中按本任务关键词读取最新且未被取代的决定，不整份吞入历史。
5. 从实时代码和配置确认当前实现；从记忆与试验记录确认验证等级。不得把旧参数、旧协议或旧架构当成当前事实。

快速接手三工程时，读取 [references/onboarding.md](references/onboarding.md)。跨工程接口或责任归属不清时，读取 [references/system-contracts.md](references/system-contracts.md)。

## 按任务加载专项经验

- 判断双核职责、实时通道、代码放置或现有任务栈：读 [references/flight-architecture.md](references/flight-architecture.md)。
- 新增、修改、组合或抢占飞行任务：读 [references/mission-development.md](references/mission-development.md)。
- 估计器、姿态、高度、光流、水平引导、混控或控制调参：读 [references/flight-control.md](references/flight-control.md)。
- 小车运动学、轮速、追灯、压灯、丢帧续航或 C5：读 [references/car-control.md](references/car-control.md)。
- 主副摄、V灯、信标、鱼眼校正、发布与接管：读 [references/vision-and-helper.md](references/vision-and-helper.md)。
- 异常定位、日志判读、软硬件边界：读 [references/diagnosis-playbook.md](references/diagnosis-playbook.md)。
- 修改代码、协议、共享内存或行为参数：读 [references/validation-matrix.md](references/validation-matrix.md)。
- 要把新经验写回此 skill：先读 [references/evolution-protocol.md](references/evolution-protocol.md)。

只读取当前任务需要的参考文件。

## 核心开发方法

沿真实因果链寻找第一处分歧：

`原始测量 -> 有效性/新鲜度 -> 估计 -> 任务目标 -> 成形命令 -> 控制输出 -> 混控/传输 -> 物理响应`

- 解释或审查保持只读；诊断只定位根因，除非用户同时要求修复。
- 一轮只验证一个主要机制，或一组不可拆分的参数；明确预期观测、回退条件和未改变边界。
- 优先修正数据真实性、状态语义和接口边界，再考虑增益与限幅。
- 正确软件命令与错误物理响应并存时，转向电机、电调、轮组、接线、安装、重心、挂点和传感器质量，不无限调软件。
- 负面试验能排除原因，也属于有价值证据；记录结论，不重复堆叠失败方案。

## 三工程不变量

- 飞控拥有飞行安全、姿态/高度/水平闭环、任务时间轴和最终 A4 发布；小车拥有底盘安全、模式仲裁、运动成形与四轮执行；副摄只提供感知证据，不直接控制无人机或小车。
- 图像坐标、飞控纠偏坐标和小车车体坐标是不同契约。每个边界只转换一次，方向错误只改对应 Adapter，不在算法链中到处翻转。
- A4/C5、B1/B2、共享内存、帧号、校验、端序、单位、新鲜度和文本遥测都是接口。改动前找全生产者与消费者，保留兼容或协调迁移。
- 真实当前帧、短时 hold、稳定发布、任务选中和屏幕显示是不同状态，不得用显示存在替代控制有效。
- 不删除或弱化 `disarm`、`estop`、`failsafe`、`MISSION_STOP`、硬零输出、链路超时或陈旧数据门控来换取响应速度。
- 高速控制路径不做阻塞外设、复杂字符串、长循环或重诊断；通讯和显示不能破坏控制节拍。

## 验证与结论

保持验证等级独立：

`已实现 -> 已构建 -> 已架空/拆桨 -> 已系留 -> 已低风险实测 -> 已在声明范围接受`

同时保留 `候选`、`未验证`、`已否决`、`已撤回`、`已取代`。构建通过不能证明飞行、行驶、识别率或通讯可靠性。

结束时说明：改了什么机制、软件验证结果、需配套构建/烧录的端点、仍待进行的物理验证、剩余风险和下一项最小判别试验。

## 自迭代边界

此 skill 可以随项目演进，但不能在普通任务中自动把猜测写成规则。只有形成经过验证、会改变未来决策的通用经验时才更新；当前参数、一次性试验和聊天过程继续留在代码、文档或项目记忆中。

三工程同处 `Motherboard_Demo` 时，规范源是其共享层 `.agents/skills/feiyue-leiqu`；各仓库副本只用于独立克隆时发现。更新后运行 `scripts/audit-skill.ps1`，再运行 `scripts/sync-mirrors.ps1` 同步三仓并复核 diff。不要自动提交或发布。
