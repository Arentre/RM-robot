# USB CDC 启动故障现场（2026-10-07）

## 烧录前读取的证据

通过 STM32CubeProgrammer 2.15、ST-Link SWD HOTPLUG 读取；未烧写固件。
Windows 只发现 ST-Link 的 COM6，没有 STM32 CDC 或未知 USB 设备。
符号地址对应本地 15:27:55 构建的 `MDK-ARM/DevC/DevC.map`。

- `RCC_AHB2ENR` = `0x00000040`：OTG FS 时钟未开启。
- `hUsbDeviceFS`（0x200011B0）前 16 字节全零。
- `xTickCount`（0x200095A4）= 0；`xSchedulerRunning` = 1。
- `ICSR` = `0x0442480E`，当前异常号 14（PendSV）。重复读取不变。
- `CFSR`、`HFSR` 均为零，未发现对应故障标志。
- `pxCurrentTCB`（0x20002298）= `0x20003B28`；任务名为 `can`。
- 该 TCB 的 `pxStack` = `0x20003920`，栈底四个字为
  `00000024 0801E347 0801C0DA 61000000`，已不是预期的 `A5A5A5A5`。
- `configCHECK_FOR_STACK_OVERFLOW = 2`；任务切换检查栈底四字，失败调用无限循环的溢出 hook。
- `xFreeBytesRemaining` 与最小历史剩余堆均为 `0x2CF8`（11512 字节）。

链接器调用图 `DevC.htm` 的 CAN 最大静态调用深度为 572 字节，原分配仅 512 字节。
结合真实栈底损坏与 PendSV 停滞，证据指向 CAN 任务栈溢出，USB 默认任务因此未执行。
尝试 halt/coreReg 未成功取得 PC；最后 DHCSR 的 S_HALT 为 0，没有留下调试暂停状态。

## 源码修复

| 任务 | 原栈（字节） | 新栈（字节） | 当前链接调用图最大深度 |
|---|---:|---:|---:|
| can | 512 | 1024 | 572 |
| defaultTask（USB 初始化） | 512 | 1024 | 408 |
| firewater（遥测及命令） | 2048 | 3072 | 1920 |

静态调用深度还需加上异常/调度上下文及可能的浮点上下文，不能按此值贴边分配。
总共增加 2048 字节任务栈，启动现场剩余堆可容纳；后续仍应观察最小剩余堆和栈水位。
溢出 hook 增加 `stack_overflow_task` 和 `stack_overflow_task_name`，可在调试器查看。
保留之前的 USB 未配置检查、发送长度检查等修复。

## 待用户编译烧录后验证

1. Rebuild 当前工程后烧录新产物，重启开发板。
2. 检查 Windows 是否枚举 STM32 CDC；端口号以实际枚举为准。
3. 如仍失败，读取新 map 对应的溢出记录、tick、USB 状态，不沿用本文件旧地址。
4. CDC 恢复后先验证遥测，再检查任务栈最小剩余量。此诊断未发送电机控制命令。

源码已修改，尚未验证新固件在板运行。ST-Link 报告目标参考电压约 2.02–2.04 V，
该读数需要结合探针连接和实际供电测量确认，尚未作为本次根因。
