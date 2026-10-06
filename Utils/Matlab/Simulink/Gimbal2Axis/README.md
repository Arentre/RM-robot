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

串口试验限制单次 Yaw 相对指令为 30°、相对使能位置的目标为 ±35°、硬停机行程为 ±40°、控制输出为 0.15（约 0.45 A 指令）。MATLAB 还限制角速度为 180°/s。串口启用后如果 1 秒没有收到 `PING`，固件自动放松。Pitch 默认关闭。

支持的命令如下：

```text
ENABLE=1                       锁存当前姿态为试验原点并使能，Pitch保持关闭
ENABLE=0 / STOP                立即转入放松模式并清除目标
PING                           续期1秒通信看门狗
HOLD                           把目标改为当前反馈角
CENTER                         回到ENABLE时锁存的Yaw/Pitch原点
YAWREL=角度                    相对当前反馈的小角度目标
YAW=角度                       绝对角度，仍受使能原点±35°限制
PITCH=0 / PITCH=1              关闭/启用Pitch；启用时先保持当前位置
PITREL=角度                    Pitch相对指令，单次±2°、原点累计±5°
YPARAM=r,b0,wc,w0,out_limit    写入运行期Yaw LADRC参数
PPARAM=r,b0,wc,w0,out_limit    写入运行期Pitch LADRC参数，输出上限最大0.10
YFF=0 / YFF=1                  关闭/启用Yaw摩擦前馈
YFFPARAM=静摩擦,库仑摩擦,死区角度,速度过渡点,输出斜率
GET                            返回状态和Yaw/Pitch参数
```

先用两组独立数据做辨识和验证，再调参数：

```matlab
idRun = run_gimbal_bench_test(s,30.0);
id = identify_gimbal_yaw(idRun);
validationRun = run_gimbal_bench_test(s,30.0);
summary = tune_gimbal_yaw(s,id.B0_rad_s2_per_command,[2 4 6 8]);
```

辨识优先调用 System Identification Toolbox 的 `iddata + tfest + compare`，拟合一阶角速度/电流模型 `Kt/(J*s+B)`；积分力矩回归作为独立交叉核对和工具箱不可用时的后备。自动调参每组做 30° 往返，依次比较 `wc=2/4/6/8`，保持 `w0=4*wc`，输出上限为 0.10。它是针对 LADRC 的受限硬件网格优化，不把非线性 LADRC 强行当作 PID 交给 PID Tuner。任何行程、角速度、电流、温度、遥测超时或固件保护异常都会发送 `STOP`。

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

Pitch 的主机限制为相对原点 ±5.5°、120°/s、0.101 归一化输出、0.45 A 反馈电流和 80°C。固件限制更靠内层，通信超时仍会在 1 秒后放松。完整顺序和停机码见 `HARDWARE_TUNING_GUIDE.md`。

GM6020 手册明确给出电流指令 ±16384 对应 ±3 A、额定电流 1.62 A、持续堵转电流 0.90 A、转矩常数 0.741 N·m/A。反馈报文只把字段称为“实际转矩电流”，没有另外标明比例；代码暂按和电流指令相同的 ±3 A 比例换算，辨识前应拿限流电源或可信电流测量做一次比例核对。

## 如果要做严格的控制器 HIL

保留 `Gimbal_Control` 与 LADRC，另加一个测试模式：把当前从 CAN/IMU 取得的反馈改为主机提供的虚拟反馈，并把控制输出发回主机而不发给真实电机。Simulink 在主机或实时目标上运行虚拟 GM6020/负载。接口必须有帧序号、时间戳、超时清零和明确的断连行为。现有 50 Hz 文本 USB 接口不能直接闭合 500 Hz 的虚拟负载回路；普通 Windows MATLAB/USB CDC 也不能保证每 2 ms 完成一次确定性往返。这部分需要单独设计和验证通信链路。
