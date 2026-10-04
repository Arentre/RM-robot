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

## 记录实机数据，保持 STM32 控制代码不变

当前固件在 `User/task/info.c` 通过 USB CDC 以约 50 Hz 输出 `yaw:` 开头的 26 通道 FireWater 文本，并接收 `YAW=角度`、`PIT=角度` 命令。MATLAB 只负责读取、画图、保存和发送测试目标；STM32 仍以 500 Hz 控制真实云台。这是实机闭环测试，不是把 Simulink 虚拟负载接到 STM32 的严格 HIL。

1. 接通云台供电、IMU、两台 GM6020、CAN 和 STM32 的 USB CDC。确认电机方向、编码器、Pitch 机械限位、急停方式。初次测试先保持 `gimbal_test_enable=0`。
2. 关闭 VOFA+ 等占用同一 COM 口的软件，在 MATLAB 输入 `serialportlist("available")` 查找串口。
3. 建立连接并先只读数据：

```matlab
s = serialport("COM7", 115200);  % 把 COM7 改成实际端口；USB CDC 的波特率通常只是主机参数
data = capture_gimbal_hardware(s, 10);
```

4. 检查输出 CSV 中 `YawFeedback_deg`、`PitchFeedback_deg`、`Mode`、`Enable`、`PitchProtect` 和两轴输出是否合理。需要实际运动时，按当前固件流程在 Keil Watch 中把 `gimbal_test_enable` 设为 `1`。原固件的 `gimbal_ladrc_pit_enable=0`，因此 Pitch 默认没有电流输出。
5. 从当前姿态附近的小目标开始，在同一个 `s` 上发送：

```matlab
writeline(s, "YAW=5")    % 单位：度；固件接受 0..360 度 Yaw 目标
data = capture_gimbal_hardware(s, 10);
writeline(s, "PIT=5")    % 仅在 Pitch 控制已启用且确认机械限位后使用
data = capture_gimbal_hardware(s, 10);
clear s                    % 释放串口
```

USB 目标值会保持有效，串口断开时固件不会自动撤销目标。因此每轮结束在 Keil Watch 中把 `gimbal_test_enable=0`，需要时切断电机供电。当前 50 Hz 文本遥测适合观察较慢的角度响应；它不含真实电机电流、MCU 时间戳，也不足以可靠辨识 2 ms 控制周期的快速动态。要做高精度系统辨识，下一步应在固件通信层补 MCU 时间戳、实测电流和更快的二进制记录。

## 如果要做严格的控制器 HIL

保留 `Gimbal_Control` 与 LADRC，另加一个测试模式：把当前从 CAN/IMU 取得的反馈改为主机提供的虚拟反馈，并把控制输出发回主机而不发给真实电机。Simulink 在主机或实时目标上运行虚拟 GM6020/负载。接口必须有帧序号、时间戳、超时清零和明确的断连行为。现有 50 Hz 文本 USB 接口不能直接闭合 500 Hz 的虚拟负载回路；普通 Windows MATLAB/USB CDC 也不能保证每 2 ms 完成一次确定性往返。这部分需要单独设计和验证通信链路。
