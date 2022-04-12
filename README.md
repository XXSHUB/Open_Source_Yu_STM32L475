# Open_Source_Yu_STM32L475_GCC
**本例程仅供学习使用，不得用于任何商业用途。**

# 一、环境介绍
系统：Window10 LTSC
[STM32CubeMX](https://www.st.com/zh/development-tools/stm32cubemx.html)：图形化界面配置基于HAL库的初始化代码
[VScode](https://code.visualstudio.com/)：辅助的代码编辑器
[Embedded IDE](https://docs.em-ide.com/#/README)：是一款适用于 8051/STM8/Cortex-M/RISC-V 的单片机开发环境。能够在 VScode 上提供 8051, STM8, Cortex-M, RISC-V 项目的开发, 编译, 烧录功能。
[ARM-GNU-Toolchain](https://link.zhihu.com/?target=https%3A//developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads)：GCC工具链
开发板：潘多拉 STM32L475  
# 二、功能介绍
点亮LED灯
# 三、调用函数
>HAL_GPIO_TogglePin()
