# Agent 云台调参操作说明

适用于本仓库的 RM-C、GM6020 双轴云台。当前是重装后的大惯量平台，旧轻载台架数据只作对比。Yaw 有滑环；Pitch 重装后需核对方向和上下机械行程。

## 参数和命令在哪

| 目的 | 入口 | 说明 |
| --- | --- | --- |
| 固件默认参数 | `User/module/control_params.c` | 双轴 LADRC、摩擦和起动助力。装车默认归一化输出上限 1.0、到位容差 0.5°。 |
| 当前控制逻辑 | `User/module/gimbal.c`、`User/component/ladrc.c`、`User/component/gimbal_ff.c` | 最终电机输出在叠加前馈后限幅；Pitch 限位仍有效。 |
| USB 调试与遥测 | `User/task/info.c` | `HELP` 查命令；`GET` 查当前运行参数；`FAULT` 查最近一次保护快照。 |
| 保护与停机 | `User/task/ctrl_gimbal.c` | 根据当前代码和故障快照定位，不能仅凭“没动”判断过流。 |
| MATLAB 采集与整定 | [MATLAB 调参说明](../Utils/Matlab/Simulink/Gimbal2Axis/README.md) | 兼容 47 列 FireWater，脚本接受串口对象 `s`。 |

USB 命令以换行结束。记录 `GET` 中的 `yp`、`pp`、`yff`、`pff` 和死区；不能把源码默认值当成设备当前值。涉及 EmberProbe 的实时读写时，按仓库 `.agents/skills` 中相应技能操作，并确认 ELF 与板上固件一致。通过 USB 或 EmberProbe 修改的普通控制参数只在 RAM 生效。

## 按顺序试验

1. **区分操作范围。** 没有当前带电试验请求时，先检查代码、已有日志和编译结果。固件编译产物为 `build/Debug/DevC.elf`；不要自动烧录、使能或改 Flash。
2. **准备有监督的试验。** 让遥控进入卸力档，退出自瞄与其他上位机控制。核对电机方向、姿态反馈和机械空间，保留现场可用的停机手段。Yaw 用 `ENABLE=YAW`；`ENABLE=1` 会开双轴。Pitch 先在卸力时托住机构，用 `PCAL=RESET`、`PCAL=LOW`、`PCAL=HIGH`、`PCAL=SHOW` 手动采两端。建议限位不会自动写入，禁止用电机堵转来寻边。
3. **采样辨识。** 在该轴不动时关闭它的摩擦前馈（`YFF=0` 或 `PFF=0`），Yaw 也关 `YDITHER=0`。Yaw 先做 2° 小阶跃，然后用 `run_gimbal_chirp_test(s,"up",0.45)` 和 `run_gimbal_chirp_test(s,"down",0.45)` 各采一组。Pitch 确认能安全运动后，从相同中间姿态做两组 `run_gimbal_pitch_bench_test(s,2,0.40)`。脚本暂时降低限幅并读回核对，试验结束发 STOP 并尝试恢复；原限幅更低时不会自动提高。
4. **验证和整定。** Yaw 调用 `identify_gimbal_yaw_chirp(up,down)`，Pitch 调用 `identify_gimbal_pitch(p1,p2)`。只有 `Reliable=true` 且 `RecommendedB0_rad_s2_per_command` 有效时，才将结果作为候选。用 `tune_gimbal_yaw` 或 `tune_gimbal_pitch` 做受限幅的阶跃比较，按到位、超调和回中误差筛选。拟合差或激励不足时，先检查丢帧、实际运动和信号方向，不凭惯量大小猜写新 `b0`。
5. **最后调摩擦。** Yaw 用 `tune_gimbal_friction(s,[0.040 0.060 0.080],[0.35 0.60])`，由无补偿基线、正反 2° 响应及复测决定是否保留候选。Pitch 通过 `PFF=0/1` 和 `PFFPARAM=` 做同样的正反向对比；方向差明显时分开检查重力和平衡。重力项 `PGRAV` 未辨识前保持关闭。

## 写入参数时核对单位

- `YPARAM=r,b0,wc,w0,out_limit`、`PPARAM=...`：`wc/w0` 为弧度每秒，`out_limit` 为 0 到 1 的归一化上限。它是允许值，不是要求电机一直输出该电流。
- `YFFPARAM=静摩擦,库仑摩擦,死区角度,速度过渡点,输出斜率`；`PFFPARAM` 同格式。前两项是归一化指令，死区用度，速度用度每秒，斜率用归一化输出每秒。
- `YDEAD/PDEAD` 用度，源码 `deadband_rad` 用弧度。起动助力上限可在调试器中看 `gimbal_yaw_breakaway_max` 和 `gimbal_pitch_breakaway_max`，置零仅关闭对应助力。
- 每次只改可解释的一组参数，记录原值和新值并读回。保存默认值前，至少保留一组独立复测的 CSV，记录轴、起始姿态、激励、峰值输出、反馈电流、到位误差和故障码。确认后再修改 `control_params.c` 并编译。

## 失败时怎么处理

先停机并读取 `FAULT`，保留失败时的原始遥测和回包；不要清除记录后自动重试。查清是主机限幅、固件保护、指令未确认、反馈停更还是机构未起动，再选择下一步。台架脚本当前会监测约 1.60 A 的反馈门槛；它独立于固件的 5 A 持续过流判停。电流反馈按当前 CAN 标度换算，不能拿它当校准仪表读数。不要为跑通试验移除温度、速度、CAN 或 Pitch 限位保护。
