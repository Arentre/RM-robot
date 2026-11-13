# RM C 板工程（CLion / ARM GCC）

已按四个机构、自瞄、CRSF 遥控、USART6 导航链路、基础外设及 USB 调试精简；详见 [代码结构与调参](docs/代码结构与调参.md)。所有 PID、LADRC 和前馈默认参数统一在 `User/module/control_params.c`。

VS Code 的编译、补全、ST-LINK 和 Codex 使用方式见 [VS Code 开发环境](docs/VSCode开发环境.md)。

目标：STM32F407（Cortex-M4F），使用 `DevC.ioc`、C 板 BSP 和 `DEV_BOARD_C` 宏。

## CLion

本机已安装 STM32CubeCLT 1.22.0。CLion 当前构建配置为 `Debug-STM32 ARM GCC`，目标为 `DevC`，构建目录为 `build/clion-debug`，调试服务器为现有的 `ST-LINK`。

点击锤子构建；连接 ST-LINK、接好 SWD 并给开发板供电后，可以使用刷写或调试按钮。尚未验证实板下载和运行。

其他电脑打开 `CMakeLists.txt`，可启用 `Debug` / `Release` CMake 预设。工具链会从 PATH、`ARM_TOOLCHAIN_ROOT/bin` 或标准 STM32CubeCLT 安装目录查找 ARM GCC。CLion 调试服务器路径需匹配本机安装。

## 命令行构建

确保 CMake、Ninja 在 PATH 中：

```sh
cmake --preset Debug
cmake --build --preset Debug
cmake --preset Release
cmake --build --preset Release
```

每次构建生成 `DevC.elf`、`DevC.hex`、`DevC.bin`、`DevC.map`；命令行预设输出在 `build/Debug` 或 `build/Release`。

## 配置维护

- `cmake/rm-board-c.cmake`：按 Keil 的 DevC 目标显式列出 User 源码，排除 A 板 BSP 和未使用设备。
- `cmake/gcc-arm-none-eabi.cmake`：ARM GCC、Cortex-M4F 硬浮点参数和 Flash 链接脚本。
- `cmake/stm32cubemx/`：CubeMX 生成的 HAL、USB、FreeRTOS 配置。
- 包含路径使用 `/` 以支持 macOS / GCC；CLI 的状态常量使用 C 枚举以满足 `case` 常量表达式要求。
- GCC 使用 packed 属性兼容原有 Keil `__packed`，并启用 newlib-nano 浮点格式化输出。

Debug、Release 和当前 CLion 构建配置均已通过编译链接。现有业务代码仍有缺少返回值、可能未初始化等编译警告；编译通过不代表已完成实板功能验证。

CLion 调试服务器配置参考：[JetBrains Debug servers](https://www.jetbrains.com/help/clion/debug-servers.html)。
