# STM32F405 固件工程

本仓库包含质量驱动无人机的 STM32F405 源代码、驱动库、Keil 工程文件，以及 EIDE/VS Code 配置。

## 目录结构

- `HAL_sourcecode_mass_actuated_uav`：主要源代码、驱动库和 Keil 工程文件。
- `HAL_sourcecode_mass_actuated_uav_eide`：EIDE、VS Code 工程配置和辅助脚本。

## 查看源代码

主要程序代码位于：

- `HAL_sourcecode_mass_actuated_uav/src`
- `HAL_sourcecode_mass_actuated_uav/lib`

Keil 工程位于：

- `HAL_sourcecode_mass_actuated_uav/project`

## 使用 EIDE 打开

使用 EIDE 或 VS Code 打开：

- `HAL_sourcecode_mass_actuated_uav_eide/Firmware_F405.code-workspace`

## 注意事项

`.cmsis`、`.pack`、`build` 和日志文件属于本地依赖或编译输出，不上传到 GitHub。
