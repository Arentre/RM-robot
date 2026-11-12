# GM6020 双轴云台：离线模型与实机数据

自动化调参前请阅读 [Agent 云台调参操作说明](../../../../docs/Agent云台调参.md)。


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

单独调 Yaw 时先发送 `ENABLE=YAW`，再发送 `yaw=目标角度`，无需循环发送目标或 `ping`。每条指令以回车或换行结束；`stop` 放松、`center` 回使能位置、`get` 查询状态。`err:YAW,disabled` 表示需要使能，`err:YAW,range` 表示数字或范围不合法。

支持的命令如下：

```text
ENABLE=1 / ENABLE=YAW          锁存当前姿态；前者启用双轴，后者仅启用 Yaw
ENABLE=0 / STOP                立即转入放松模式并清除目标
PING                           兼容旧脚本，无操作；保持目标不需要心跳
HOLD                           把目标改为当前反馈角
CENTER                         回到ENABLE时锁存的Yaw/Pitch原点
YAWREL=角度                    相对当前反馈的小角度目标
YAW=角度 / YAWABS=角度         IMU 绝对角度，按一圈归一化
PITCH=0 / PITCH=1              关闭/启用Pitch；启用时先保持当前位置
PITREL=角度                    Pitch 相对目标，最终 IMU 目标需在 ±70° 内
PIT=角度 / PITABS=角度         Pitch IMU 绝对目标
YPARAM=r,b0,wc,w0,out_limit    写入运行期Yaw LADRC参数
PPARAM=r,b0,wc,w0,out_limit    写入运行期 Pitch LADRC 参数
YFF=0 / YFF=1                  关闭/启用Yaw摩擦前馈
YFFPARAM=静摩擦,库仑摩擦,死区角度,速度过渡点,输出斜率
YCHIRP=UP / YCHIRP=DOWN        STM32内部生成±5°、0.2～2 Hz扫频
YDITHER=0 / YDITHER=1          关闭/启用小角度防粘滞微扰
YDITHERPARAM=幅值,频率Hz        默认0.008、25 Hz
GET / FAULT / IMU              查询控制状态、故障快照、IMU
HELP / PROTO?                  查看命令提示 / 查询二进制协议
GCAL                           停机静止时校准陀螺仪零偏
PCAL=RESET/LOW/HIGH/SHOW        卸力后手动采集 Pitch 两端并查看建议限位
YDEAD=角度 / PDEAD=角度         修改控制死区
PFF=0/1 / PFFPARAM=五个数       Pitch 摩擦补偿（字段同 YFFPARAM）
PGRAV=0/1 / PGRAVPARAM=幅值,相位度  Pitch 重力补偿
PFILTER=Hz                     Pitch 输出滤波，0 为关闭
```

Yaw 试验用 `ENABLE=YAW`，须先烧录支持该命令的新固件。装车默认输出上限仍为 **1.0**；采集时临时取现有上限与试验预算的较小值，试验结束或退出后发 STOP 并尝试恢复原设置。Yaw 阶跃/扫频预算默认 **0.45**，Pitch 阶跃默认 **0.40**（同时将保持姿态的 Yaw 限在 **0.30**）。脚本对两轴保留 **1.60 A 反馈值**、速度、行程、温度及遥测保护；这与固件 5 A 判停规则独立，反馈电流标度仍待校准。

先做 2° 单轴检查。辨识时关闭被测轴摩擦前馈和微扰，以减少附加激励。在线调参不会自动保存到 Flash；实机验证后再写入 `User/module/control_params.c`。

```matlab
gimbal_usb_command(s,'STOP','ack:STOP');
gimbal_usb_command(s,'YFF=0','ack:YFF=0');
gimbal_usb_command(s,'YDITHER=0','ack:YDITHER=0');
trial = run_gimbal_bench_test(s,2);
up = run_gimbal_chirp_test(s,"up",0.45);
down = run_gimbal_chirp_test(s,"down",0.45);
id = identify_gimbal_yaw_chirp(up,down);
if id.Reliable
    yawSummary = tune_gimbal_yaw(s,id.RecommendedB0_rad_s2_per_command,[2 4 6 8],5,0.40);
end
frictionSummary = tune_gimbal_friction(s,[0.040 0.060 0.080],[0.35 0.60]);
```

采集脚本兼容 35、41、43、47 列 FireWater 数据。新版辨识以实际 CAN 归一化指令与角速度为输入，对原始 MCU 时间戳做分段积分，加入阻尼、库仑摩擦和恒定扰动项，丢帧间隔不跨段拟合。估计与验证使用不同采集，两次交叉拟合、增益符号和一致性都达标才推荐经验 `b0`；不输出未经电流标定的物理惯量。单组阶跃可用 `identify_gimbal_yaw(trial)` 查看诊断，两组独立阶跃也可传入 `identify_gimbal_yaw(est,validation)`；短阶跃的数据可能不足，优先用扫频。历史 `tfest` 模型仍可用 `evaluate_gimbal_chirp_models` 对比，历史台架结果不作为新平台参数。

LADRC 扫参默认做 5° 阶跃，不再用 30° 作为每次试验的起点；没到位或回中不合格的候选不入选。摩擦扫参用无补偿基线、正反向 ±2° 和独立复测，未确认改善时恢复原参数。固件在 Yaw 向目标运动且控制器已经反向刹车时撤掉摩擦前馈。所选参数仅更新运行时值；确认实板效果后再写入源码。若试验触发安全阈值，脚本停机并退出，不自动重试。

Pitch 在确认方向和机械限位后，从远离限位的相同姿态开始做两组独立试验。原先 0.5° 阶跃已落入到位区，改用 2°；Pitch 辨识也改用实际归一化指令，近似用常量项吸收同一姿态附近的重力偏置。限位内没有足够有效运动时，收集更多同一姿态附近的试验数据；不接受单次采集直接给出的增益。

```matlab
gimbal_usb_command(s,'PFF=0','ack:PFF=0');
p1 = run_gimbal_pitch_bench_test(s,2,0.40);
p2 = run_gimbal_pitch_bench_test(s,2,0.40);
pid = identify_gimbal_pitch(p1,p2);
if pid.Reliable
    pitchSummary = tune_gimbal_pitch(s,pid.RecommendedB0_rad_s2_per_command,[1 2 3 4],2,0.35);
end
```

GM6020 手册明确给出电流指令 ±16384 对应 ±3 A、额定电流 1.62 A、持续堵转电流 0.90 A、转矩常数 0.741 N·m/A。反馈报文只把字段称为“实际转矩电流”，没有另外标明比例；代码暂按和电流指令相同的 ±3 A 比例换算，辨识前应拿限流电源或可信电流测量做一次比例核对。

## 如果要做严格的控制器 HIL

保留 `Gimbal_Control` 与 LADRC，另加一个测试模式：把当前从 CAN/IMU 取得的反馈改为主机提供的虚拟反馈，并把控制输出发回主机而不发给真实电机。Simulink 在主机或实时目标上运行虚拟 GM6020/负载。接口必须有帧序号、时间戳、超时清零和明确的断连行为。现有 50 Hz 文本 USB 接口不能直接闭合 500 Hz 的虚拟负载回路；普通 Windows MATLAB/USB CDC 也不能保证每 2 ms 完成一次确定性往返。这部分需要单独设计和验证通信链路。
