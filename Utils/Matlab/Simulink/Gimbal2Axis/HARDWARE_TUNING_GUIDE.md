# GM6020 双轴云台实物辨识与调参步骤

本文对应当前工程的 USB CDC 安全试验固件和 MATLAB R2024a 脚本。编译、烧录由操作者完成；MATLAB 试验在烧录成功后进行。

## 1. 本次固件提供的保护

- 上电默认 `RELAX`，没有电机电流指令。
- `ENABLE=1` 把当前姿态锁存为试验原点，Yaw 使能，Pitch 保持关闭。
- 串口启用后必须持续收到 `PING`；超过 1 秒自动放松。
- Yaw 单条相对指令最大 30°，目标累计最大 ±35°，实际行程超过 ±40°停机。
- Pitch 单条相对指令最大 2°，目标累计最大 ±5°，实际行程超过 ±7°停机。
- 任一已启用轴角速度超过约 200°/s、反馈不是有限值或 Pitch 超过 ±70°都会停机。
- 串口模式下 Yaw 输出最大 0.15，约等于 0.45 A 指令；Pitch 最大 0.10，约等于 0.30 A 指令。
- MATLAB Yaw 阶跃默认 30°，仍保持 180°/s、0.45 A 反馈和 0.15 输出限制；Pitch 默认 0.5°。异常时立即发 `STOP`。

停机码：

| 码 | 含义 |
|---:|---|
| 0 | 无故障 |
| 1 | 收到 `STOP` 或 `ENABLE=0` |
| 2 | 串口看门狗超时 |
| 3 | Yaw 实际行程超限 |
| 4 | 角速度超限 |
| 5 | IMU 反馈无效 |
| 6 | Pitch 绝对角保护 |
| 7 | Pitch 实际行程超限 |

## 2. 编译和烧录前

需要纳入 Keil 工程并重新编译的源码是：

- `User/device/can.c`、`User/device/can.h`
- `User/module/gimbal.c`、`User/module/gimbal.h`
- `User/task/ctrl_gimbal.c`
- `User/task/info.c`

编译时确认没有新增错误；烧录后先不要打开 VOFA+，避免 COM 口被占用。机械上保证 Yaw 原点两边至少各有 45°空余，线束不会被拉紧；Pitch 负载必须可由手托住。准备随时断开电机电源。

GM6020 手册给出的电流指令范围是 `-16384..16384` 对应 `-3..3 A`。反馈报文只说明字段为“实际转矩电流”，未单独给出比例。本工程暂按同样的 ±3 A 比例换算，因此实物辨识结果必须再和可信电流测量核对。

## 3. 烧录后的只读预检

在 MATLAB 命令窗口执行：

```matlab
cd('D:\Desktop\RM电控\全向轮哨兵\Utils\Matlab\Simulink\Gimbal2Axis')
serialportlist("available")
s = serialport("COM7",115200);   % 改成实际端口
report = preflight_gimbal_hardware(s,3);
```

已经有 `s` 时直接传对象，不要再次写 `serialport(s,115200)`。预检先发送 `STOP`，只读取 35 通道遥测，不会使能电机。它检查：新固件字段、两轴未使能、停止状态输出、静止角速度、CAN 温度反馈、电流反馈和 MCU 遥测频率。

如果端口被占用：

```matlab
clear s
```

关闭 VOFA+、串口助手或其他 MATLAB 会话后再重新创建连接。

## 4. Yaw 小角度检查和辨识

先让线束有余量并把 Pitch 支撑好：

```matlab
signCheck = run_gimbal_bench_test(s,2.0); % 首次只确认方向和闭环极性
yawRun1 = run_gimbal_bench_test(s,30.0);
yawId1 = identify_gimbal_yaw(yawRun1);
yawRun2 = run_gimbal_bench_test(s,30.0);
yawId2 = identify_gimbal_yaw(yawRun2);
```

只有首次方向检查使用 2°。确认目标和反馈同向后，正式辨识过程是：原位保持 1.5 秒、Yaw +30° 保持 4.5 秒、`CENTER` 精确回到启用原点并保持 4.5 秒、`HOLD`、`STOP`。曲线和 CSV 自动保存在当前目录。

检查以下结果：

- 目标与反馈方向一致，没有向相反方向加速。
- `YawTravel_deg` 不超过 ±35°，角速度、电流和温度没有触发保护。
- 两次 `J_kg_m2`、`B_Nm_s_rad` 均为正，`B0_rad_s2_per_command` 差异建议小于 30%。
- `ValidationRelativeRmse` 越小越好，首轮建议小于 0.20。恒定输入时 `ValidationR2` 可能没有统计意义，应结合相对 RMSE 和曲线判断。

辨识首先使用 System Identification Toolbox：把正向阶跃和回中数据重采样成 `iddata`，通过 `tfest` 拟合一阶角速度/电流传递函数 `Kt/(J*s+B)`，再用 `compare` 做独立阶段验证。同时使用下式的积分回归交叉核对：

```text
Kt * (i - i_bias) = J * dω/dt + B * ω
Kt = 0.741 N·m/A
b0 = Kt * 3 / J
```

## 5. Yaw 摩擦补偿

2026-10-06 的 30°实测显示：正向达到约 5°/s 时电流约 0.129 A、输出约 0.0445；反向分别约 0.119 A、0.0411。运动段输出中位数约 0.030。固件加入连续的静摩擦/库仑摩擦前馈：误差小于死区时为零，静止启动时使用较大的静摩擦项，速度升高后平滑过渡到库仑摩擦项；换向时按可调斜率平滑穿过零，并继续受总输出 0.15 限幅。

前馈经过本机独立验证后在源码中默认启用；可先用 `YFF=0` 临时关闭做 A/B 对比。重新编译烧录后执行：

```matlab
frictionSummary = tune_gimbal_friction(s,[0.025 0.035 0.040]);
```

脚本每组只做受保护的 2°试验，比较启动延迟、目标幅值误差、超调、末端误差、峰值速度和电流。幅值误差会同时惩罚超调和欠调，避免把没有到达 2°目标的参数误判为最好。选中值只写入 RAM。串口命令：

```text
YFF=0 / YFF=1
YFFPARAM=静摩擦输出,库仑摩擦输出,误差死区deg,速度过渡deg/s,输出斜率/s
```

本机 2026-10-06 独立验证后采用的 Yaw 参数为：静摩擦 `0.035`、库仑摩擦 `0.020`、死区 `0.2°`、速度过渡点 `5°/s`、输出斜率 `0.12/s`。2°测试的峰值行程由补偿前约 `4.04°` 降到 `1.97°`，峰值速度由约 `81.1°/s` 降到 `25.5°/s`，峰值反馈电流由约 `0.144 A` 降到 `0.078 A`。30°独立验证达到约 `29.66°`，峰值速度约 `82.3°/s`，回中后残余行程约 `0.047°`，没有触发保护。

早期30°单阶跃数据的 `tfest` 独立验证拟合为负。后续改用 STM32 500 Hz控制任务内部生成的 `YCHIRP=UP/DOWN`：幅值 ±5°、频率 `0.2～2 Hz`、时间20秒。上扫频和下扫频分别用于估计与验证。闭环三阶模型双向验证约为 `70.7%/66.3%`；电流到角速度模型两次得到的 `b0≈182.5` 差异约 `0.054%`，但开环残差仍与输入相关。把该值试写 LADRC 后触发角速度保护，因此最终不采用，继续使用 `b0=500`。

## 6. Yaw 运行时扫参

扫频采集和模型比较：

```matlab
up = run_gimbal_chirp_test(s,"up");
down = run_gimbal_chirp_test(s,"down");
idResult = identify_gimbal_yaw_chirp(up,down);
modelSummary = evaluate_gimbal_chirp_models(up,down);
```

最终固定 `b0=500`，通过受保护的30°硬件试验逐级比较 `r/wc/w0`。最终参数：

```text
r=10, b0=500, wc=12, w0=48, out_limit=0.10
```

两次独立30°复测：上升时间 `0.260～0.276 s`，超调 `0.36～0.76%`，峰值速度约 `140°/s`，峰值电流 `<0.18 A`，无保护停机。原参数上升时间约 `0.628 s`。

通用运行时扫参命令仍可使用：

```matlab
yawSummary = tune_gimbal_yaw(s,yawId2.B0_rad_s2_per_command,[2 4 6 8]);
```

脚本逐项写入 `YPARAM=r,b0,wc,w0,out_limit`，每组做一次受保护的 30° 往返，并按末端误差、超调、峰值角速度和电流评分。因为 LADRC 含跟踪微分器和扩张状态观测器，`pidtune` 不能直接完整整定它；专业模型用于判断结构、重复性和候选范围，最终参数必须通过受约束硬件试验选择。选择结果只保存在 RAM；断电后恢复源码默认值。

小角度防粘滞微扰默认仅在误差 `0.2～1.5°`、速度低于 `1°/s`时启用，默认幅值 `0.008`、频率 `25 Hz`；电机开始运动即撤掉。可用 `YDITHER=0/1` 和 `YDITHERPARAM=幅值,频率` 临时调整。它在最终两次30°复测中没有引起振荡。

参数含义：

| 参数 | 含义 | 调大后的主要影响 |
|---|---|---|
| `r` | 跟踪微分器速度 | 目标变化更快，过大时指令更激烈 |
| `b0` | 被控对象输入增益估计 | 必须优先由惯量估计，不用于盲目追速度 |
| `wc` | 控制带宽 | 响应更快，噪声、电流和振荡风险上升 |
| `w0` | 扰动观测器带宽 | 扰动估计更快，过大时放大测量噪声 |
| `out_limit` | 归一化电流指令限制 | `1.0` 约对应 3 A；试验保持 0.10 或更低 |

## 7. Pitch 辨识与调参

Yaw 完成且状态稳定后，手托住 Pitch 负载再执行：

```matlab
pitchRun1 = run_gimbal_pitch_bench_test(s,0.5);
pitchId1 = identify_gimbal_pitch(pitchRun1);
pitchRun2 = run_gimbal_pitch_bench_test(s,0.5);
pitchId2 = identify_gimbal_pitch(pitchRun2);
pitchSummary = tune_gimbal_pitch(s,pitchId2.B0_rad_s2_per_command,[1 2 3 4]);
```

Pitch 脚本发送 `PITCH=1` 后才启用 Pitch LADRC。辨识中的 `BaselineCurrent_A` 同时包含重力补偿、装配偏置和电流零偏。由于角度变化只有 0.5°，算法把这段范围内的重力矩近似为常量。

如果 Pitch 使能后缓慢下坠，保持托住负载并让保护或 `STOP` 放松，不要直接提高输出上限。先检查重心、方向、电流比例和基线电流，再决定是否扩大试验范围。

## 8. 固化参数

确认独立验证数据也稳定后，把选出的 Yaw/Pitch 参数写回 `User/module/gimbal.c` 中：

```c
gimbal_ladrc_yaw_param
gimbal_ladrc_pit_param
```

随后由操作者重新编译、烧录，再运行一次只读预检和小角度验证。不要直接把扫参中的高带宽组写入源码而跳过独立验证。

## 9. 手动串口命令

```text
STOP / ENABLE=0
ENABLE=1
PING
HOLD
CENTER
GET
YAWREL=1
YAW=绝对角度
PITCH=0 / PITCH=1
PITREL=0.5
YPARAM=r,b0,wc,w0,out_limit
PPARAM=r,b0,wc,w0,out_limit
YFF=0 / YFF=1
YFFPARAM=静摩擦,库仑摩擦,死区角度,速度过渡点,输出斜率
```

优先使用 `YAWREL`、`CENTER` 和 `PITREL`。Yaw 绝对角命令也受启用原点 ±35°限制，从而避免 0°/360°附近因显示套圈造成意外的大角度动作。
