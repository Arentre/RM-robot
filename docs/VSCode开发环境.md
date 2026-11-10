# VS Code：RM-C 开发环境

## 已配置的组成

| 能力 | 组件 |
| --- | --- |
| C/C++ 语义补全、跳转、引用、参数提示 | clangd，使用 macOS 已安装的 `/usr/bin/clangd` |
| CMake 预设和构建目标 | Microsoft CMake Tools |
| STM32 下载、断点、寄存器和内存查看 | Cortex-Debug 及其依赖扩展 |
| AI 对话和代码修改 | OpenAI 官方 Codex（`openai.chatgpt`） |

已移除工作区原先指向旧 Windows/K230 工程的配置。CLion 配置仍保留，两边共享同一套 CMake、ARM GCC、源码和 Debug/Release 预设。

## 日常操作

1. 用 VS Code 打开整个 `RM-robot` 文件夹。
2. CMake 配置预设选择 `Debug`。首次打开时运行配置，生成 `build/Debug/compile_commands.json`。
3. 按 **⌘⇧B**，运行 `RM-C: build Debug`。任务会先配置再编译，输出 `build/Debug/DevC.elf/.hex/.bin`。
4. 按 **⌘⇧P**，执行 `Tasks: Run Task`，可选择 `RM-C: build Release`。
5. 打开“运行和调试”，选择 `RM-C: ST-LINK 下载并调试`，连接 ST-LINK 并给开发板供电后按 **F5**；该配置会编译、下载固件并在 `main` 停下。
6. 调试已经烧录的程序时，选择 `RM-C: ST-LINK 附加调试（不下载）`。它仍会在本机编译，以提供 ELF 符号，但不执行固件下载。
7. 点击右侧 **Codex** 标签进行 AI 对话。与 VS Code 自带的 Copilot 聊天入口不同；本次没有开通任何新订阅。

## 补全为什么这样配置

`.clangd` 固定读取 `build/Debug` 的编译数据库；clangd 从实际构建命令获得 `DEV_BOARD_C`、`STM32F407xx`、头文件目录和 Cortex-M4F 参数。`--query-driver` 仅允许查询本机 STM32CubeCLT 安装目录中的 ARM 编译器，以找到正确的交叉编译标准头文件。

这是语义补全，与 AI 自动续写不同。不要同时启用 clangd 和 Microsoft C/C++ 的 IntelliSense；工作区设置已禁用后者的语言引擎，避免重复提示。

可使用 **F12** 跳转定义、**⇧F12** 查看引用、**⌃Space** 请求补全（若 macOS 把该键用于切换输入法，可改用命令面板的 `Trigger Suggest`）、**⇧⌘Space** 显示参数提示。

修改头文件目录、宏或新增 CMake 源文件后重新运行配置。补全失效时先检查编译数据库是否存在，再执行 `clangd: Restart language server`。工作区默认按 Debug 条件解析，Release 构建不会自动切换 clangd 的数据库。

## 文件入口

- `.vscode/settings.json`：扩展、工具路径与编辑器设置。
- `.vscode/tasks.json`：Debug/Release 的配置及构建任务。
- `.vscode/launch.json`：ST-LINK 下载/附加调试。
- `.vscode/extensions.json`：推荐扩展。
- `.clangd`：语义解析配置。

本机 STM32CubeCLT 根目录为 `/opt/ST/STM32CubeCLT_1.22.0`。换电脑或升级 CLT 后，需要更新 VS Code 用户设置中的工具路径，以及 `.vscode/launch.json` 中显式填写的路径；任务引用设置中的 CMake 路径。不会修改系统默认编译器。

## 验证范围

已在 VS Code 内执行 Debug 构建任务，配置和构建均成功。clangd 对底盘模块和云台任务均完成检查，0 个解析错误；Cortex-Debug 配置字段已对照本机安装扩展的 schema 验证，GDB、ST-LINK、CubeProgrammer、SVD 路径均存在。Codex 侧栏已显示，并识别到现有账号和聊天列表。

尚未对开发板进行下载或实机调试验证。clangd 能补齐交叉编译上下文，但不能保证每一种补全、重构和调试体验都等同于 CLion。

## 编辑器选择

按当前“重视 AI 参与”的需求，建议先以 VS Code + Codex 为主要编辑环境，保留 CLion 用于对比导航、复杂重构和实板调试体验。两者可以同时安装、打开同一工程；切换编辑器无需维护两份代码。避免两个编辑器同时改写同一个文件或同时启动针对同一开发板的调试会话。

参考：[clangd 编译命令说明](https://clangd.llvm.org/design/compile-commands)、[Cortex-Debug 配置](https://github.com/Marus/cortex-debug/blob/master/debug_attributes.md)、[OpenAI Codex IDE 官方文档](https://learn.chatgpt.com/docs/codex/ide)。

## USB DFU 图形按钮

已安装 Task Buttons（`spencerwmiles.vscode-task-buttons`）和 Homebrew `dfu-util`，VS Code 底部状态栏提供四个按钮：

- **编译 Debug**：配置并编译，生成 `build/Debug/DevC.bin`。
- **全量编译 Debug**：重新配置后，使用 `--clean-first` 清理构建产物并重新编译整个 Debug 工程。
- **检测 DFU**：只列出 ST 系统 DFU 设备（USB ID `0483:df11`），不写入固件。
- **DFU 烧录**：先编译，成功后将最新的 Debug BIN 写入 `0x08000000`，并请求退出 DFU。该地址与当前链接脚本一致。

使用时，按 RM-C 用户手册设置 BOOT 并复位进入 DFU，通过 USB 数据线连接 Mac，点击“检测 DFU”确认识别，再点击“DFU 烧录”。烧录后恢复正常 BOOT 配置，必要时手动复位。每次通过系统 Bootloader 烧录前都需进入 DFU 模式；当前工程正常运行时的 USB CDC 串口不等于 DFU。

如果按钮没有立即出现，执行命令面板的 `Developer: Reload Window`。也可通过 `Tasks: Run Task` 运行对应的 `RM-C` 任务。

烧录任务限定 ST 系统 DFU USB ID；使用时仅连接一块待烧录的 DFU 板。工具路径为 `/opt/homebrew/bin/dfu-util`，换电脑后需相应调整 `.vscode/tasks.json`。此流程不依赖 STM32CubeProgrammer，断点调试继续使用 ST-LINK。配置时未执行实际烧录。

## 跨工程共用本机环境

本机的 CMake、Ninja 环境、clangd 和 Cortex-Debug 工具路径已放到 VS Code 用户设置中，编译按钮和对应任务也已加入用户设置与用户任务。新工程无需复制 RM-C 的 `.vscode` 目录，即可使用全局的“编译 Debug”和“全量编译 Debug”按钮，前提是工程自身提供名为 `Debug` 的 CMake 配置预设及构建预设。RM-C 仍有自己的 DFU 按钮。

用户配置位于 `~/Library/Application Support/Code/User/settings.json` 和同目录的 `tasks.json`。工作区的同名设置优先于用户设置；老工程里的错误路径可能需要删除。CMake Tools 会把编译数据库复制到工程根目录供 clangd 使用；若只通过任务编译的新工程无法补全，可先执行 CMake Tools 的 Configure，或按该工程的构建目录指定编译数据库。

共享环境不替代芯片配置。H743 工程必须使用自己的 Cortex-M7 工具链参数、启动文件、内存布局及 H7 调试目标，不能复制 RM-C 的 Cortex-M4/F407 链接脚本。Keil/CubeIDE 工程若尚无 CMake 构建入口，仍需完成首次适配，之后切换不必重复配置工具。DFU 固件路径与写入地址暂保留在各工程中。
