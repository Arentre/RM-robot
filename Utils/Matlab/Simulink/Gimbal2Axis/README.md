# GM6020 双轴云台：离线模型与实机数据

适用 MATLAB/Simulink R2024a。`gimbal_2axis.slx` 是 500 Hz 数值仿真，云台惯量、摩擦及 Pitch 重力矩仍为假设值。模型已移除实时 3D 和仿真节奏限制。

## 运行离线仿真

在 MATLAB 命令窗口执行：

```matlab
cd('D:\Desktop\RM电控\全向轮哨兵\Utils\Matlab\Simulink\Gimbal2Axis')
build_gimbal_sim             % 重新生成 .slx，恢复脚本中的默认参数
out = run_gimbal_sim;        % 运行并生成曲线和指标
open_system('gimbal_2axis.slx')
```

`gimbal_response.png` 按 Yaw/Pitch 分列展示目标与反馈角度、跟踪误差、角速度、电流与归一化指令、电机转矩与外扰。`gimbal_metrics.csv` 含末端误差、扰动前 RMS/峰值误差、峰值角速度、电流与转矩、90% 上升时间、5% 稳定时间和超调。稳定时间为空表示在外扰到来前未稳定。模型中 Angle、Current、Rate scope 可直接观察原始信号。电流是理想电流环模型的结果，不是实测电流。

模型工作区的 `P` 保存参数。`Kt=0.741 N·m/A`、`Imax=3 A` 取自 GM6020 使用说明；`Jyaw`、`Jpitch`、`Byaw`、`Bpitch`、`pitchGravity` 是演示用假设值。Yaw 的 LADRC 参数来自当前固件，Pitch 在离线模型里打开以演示双轴闭环；固件默认关闭 Pitch。这个模型没有复现 RTOS、CAN、IMU、模式切换和电流环的全部行为。

## 实机串口试验

更新后的固件在 `User/task/info.c` 通过 USB CDC 输出 `yaw:` 开头的 35 通道文本。新增 MCU 时间、GM6020 转矩电流、温度、停机原因、使能原点、相对行程和 Yaw 摩擦前馈输出。当前实测遥测与控制频率均约 500 Hz。

1. 接通云台供电、IMU、两台 GM6020、CAN 和 STM32 的 USB CDC。确认电机方向、编码器、机械限位和断电方式。
2. 关闭 VOFA+ 等占用同一 COM 口的软件，在 MATLAB 输入 `serialportlist("available")` 查找串口。
3. 建立连接并先只读数据：

```matlab
s = serialport("COM7", 115200);  % 把 COM7 改成实际端口；USB CDC 的波特率通常只是主机参数
data = capture_gimbal_hardware(s, 10);
```

4. 烧录更新后的固件后，先只读 3 秒，确认角度、电流和温度有限且合理。然后运行受保护的 30° Yaw 往返试验：

```matlab
report = preflight_gimbal_hardware(s,3);  % 始终保持STOP，只读检查
signCheck = run_gimbal_bench_test(s,2.0); % 只在首次烧录后确认方向
trial = run_gimbal_bench_test(s,30.0);
id = identify_gimbal_yaw(trial);
```

串口试验限制单次 Yaw 相对指令为 30°、相对使能位置的目标为 ±35°、硬停机行程为 ±40°、控制输出为 0.15（约 0.45 A 指令）。MATLAB 还限制角速度为 180°/s。串口目标持续保持，无需 `PING`；串口静默或 USB 断开不会自动放松，需发送 `STOP` 或由运动保护停机。Pitch 默认关闭。

命令不区分大小写，可直接发送 `yaw=30` 或 `yawabs=30`。两者均指定 IMU 坐标中的绝对角度；重复发送不会累加。接受 -180°～360°，回执显示为 [0°,360°)。例如使能时 Yaw 为 350°，发送 `yaw=10` 是跨零点正转 20°；使能时为 100°，发送 `yaw=10` 会因超过 ±35°测试行程而被拒绝。IMU 零点不代表地理北向，也不保证掉电后复现。

固件 Yaw 超速停机阈值为 6 rad/s（约 344°/s），Pitch 为 3.5 rad/s（约 200°/s）。这是故障停机阈值，不是目标转速；MATLAB 阶跃脚本仍有独立的 180°/s 限制。

手动操作先发送 `enable=1`，再发送 `yaw=目标角度`，无需循环发送目标或 `ping`。每条指令以回车或换行结束；`stop` 放松、`center` 回使能位置、`get` 查询状态。`err:YAW,disabled` 表示需要使能，`err:YAW,travel` 表示目标超出行程，`err:YAW,range` 表示数字或范围不合法。

支持的命令如下：

```text
ENABLE=1                       锁存当前姿态为试验原点并使能，Pitch保持关闭
ENABLE=0 / STOP                立即转入放松模式并清除目标
PING                           兼容旧脚本，记录通信时间；保持目标不需要心跳
HOLD                           把目标改为当前反馈角
CENTER                         回到ENABLE时锁存的Yaw/Pitch原点
YAWREL=角度                    相对当前反馈的小角度目标
YAW=角度 / YAWABS=角度         IMU绝对角度，仍受使能原点±35°限制
PITCH=0 / PITCH=1              关闭/启用Pitch；启用时先保持当前位置
PITREL=角度                    Pitch相对指令，单次±2°、原点累计±5°
YPARAM=r,b0,wc,w0,out_limit    写入运行期Yaw LADRC参数
PPARAM=r,b0,wc,w0,out_limit    写入运行期Pitch LADRC参数，输出上限最大0.10
YFF=0 / YFF=1                  关闭/启用Yaw摩擦前馈
YFFPARAM=静摩擦,库仑摩擦,死区角度,速度过渡点,输出斜率
YCHIRP=UP / YCHIRP=DOWN        STM32内部生成±5°、0.2～2 Hz扫频
YDITHER=0 / YDITHER=1          关闭/启用小角度防粘滞微扰
YDITHERPARAM=幅值,频率Hz        默认0.008、25 Hz
GET                            返回状态和Yaw/Pitch参数
```

先用两组独立数据做辨识和验证，再调参数：

```matlab
idRun = run_gimbal_bench_test(s,30.0);
id = identify_gimbal_yaw(idRun);
validationRun = run_gimbal_bench_test(s,30.0);
summary = tune_gimbal_yaw(s,id.B0_rad_s2_per_command,[2 4 6 8]);
```

宽频辨识使用 STM32 控制任务内部生成的两组独立扫频：一组 `0.2→2 Hz`，一组 `2→0.2 Hz`，幅值均为 ±5°。MATLAB 使用 System Identification Toolbox 的 `iddata + tfest + compare` 比较开环物理模型和闭环经验模型。实测闭环三阶模型的双向独立验证约为 `70.7%/66.3%`。电流到角速度模型两次得到的 `b0≈182.5` 差异仅约 `0.054%`，但因摩擦非线性和闭环残差相关，不能直接写入 LADRC；实机试写后触发角速度保护，因此保留 `b0=500`。

最终 Yaw LADRC 参数为 `r=10、b0=500、wc=12、w0=48、out_limit=0.10`。两次30°独立复测的上升时间为 `0.260～0.276 s`，超调 `0.36～0.76%`，峰值速度约 `140°/s`，峰值反馈电流低于 `0.18 A`，均未触发保护。与原始约 `0.628 s` 相比，上升时间缩短约56%。任何行程、角速度、电流、温度、遥测超时或固件保护异常仍会发送 `STOP`。

实测显示 Yaw 正反向启动力约为 0.04 归一化输出，运动摩擦约为 0.03。固件中的摩擦前馈经 2°和 30°独立验证后默认启用，参数为：静摩擦 0.035、库仑摩擦 0.020、误差死区 0.2°、速度过渡点 5°/s、输出斜率 0.12/s。换向时前馈按斜率平滑穿过零，进入误差死区则立即撤掉；可用 `YFF=0` 临时关闭做 A/B 对比。重新烧录后运行：

```matlab
frictionSummary = tune_gimbal_friction(s,[0.025 0.035 0.040]);
```

Yaw 验证完成后才能进入 Pitch。用手托住负载，先运行 0.5° 往返：

```matlab
pitchRun = run_gimbal_pitch_bench_test(s,0.5);
pitchId = identify_gimbal_pitch(pitchRun);
pitchSummary = tune_gimbal_pitch(s,pitchId.B0_rad_s2_per_command,[1 2 3 4]);
```

Pitch 的主机限制为相对原点 ±5.5°、120°/s、0.101 归一化输出、0.45 A 反馈电流和 80°C。固件保留运动保护，但不再因通信超时放松；已启用的轴会持续保持目标。完整顺序和停机码见 `HARDWARE_TUNING_GUIDE.md`。

GM6020 手册明确给出电流指令 ±16384 对应 ±3 A、额定电流 1.62 A、持续堵转电流 0.90 A、转矩常数 0.741 N·m/A。反馈报文只把字段称为“实际转矩电流”，没有另外标明比例；代码暂按和电流指令相同的 ±3 A 比例换算，辨识前应拿限流电源或可信电流测量做一次比例核对。

## 如果要做严格的控制器 HIL

保留 `Gimbal_Control` 与 LADRC，另加一个测试模式：把当前从 CAN/IMU 取得的反馈改为主机提供的虚拟反馈，并把控制输出发回主机而不发给真实电机。Simulink 在主机或实时目标上运行虚拟 GM6020/负载。接口必须有帧序号、时间戳、超时清零和明确的断连行为。现有 50 Hz 文本 USB 接口不能直接闭合 500 Hz 的虚拟负载回路；普通 Windows MATLAB/USB CDC 也不能保证每 2 ms 完成一次确定性往返。这部分需要单独设计和验证通信链路。
