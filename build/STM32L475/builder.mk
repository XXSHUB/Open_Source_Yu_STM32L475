EIDE_UNIFY_BUILDER := 1
CFLAGS := -c --apcs=interwork --cpu Cortex-M4.fp --c99 -D__MICROLIB -O0 --split_sections --diag_suppress=1 --diag_suppress=1295 -g -I.\Core\Inc -I.\Drivers\STM32L4xx_HAL_Driver\Inc -I.\Drivers\STM32L4xx_HAL_Driver\Inc\Legacy -I.\Drivers\CMSIS\Device\ST\STM32L4xx\Include -I.\Drivers\CMSIS\Include -I.\.cmsis\include -I.\MDK-ARM\RTE\_STM32L475 -I.\.eide\deps -DUSE_HAL_DRIVER -DSTM32L475xx
CXXFLAGS := -c --cpp --apcs=interwork --cpu Cortex-M4.fp -D__MICROLIB -O0 --split_sections --diag_suppress=1 --diag_suppress=1295 -g -I.\Core\Inc -I.\Drivers\STM32L4xx_HAL_Driver\Inc -I.\Drivers\STM32L4xx_HAL_Driver\Inc\Legacy -I.\Drivers\CMSIS\Device\ST\STM32L4xx\Include -I.\Drivers\CMSIS\Include -I.\.cmsis\include -I.\MDK-ARM\RTE\_STM32L475 -I.\.eide\deps -DUSE_HAL_DRIVER -DSTM32L475xx
ASMFLAGS := --apcs=interwork --cpu Cortex-M4.fp --pd "__MICROLIB SETA 1" -g -I.\Core\Inc -I.\Drivers\STM32L4xx_HAL_Driver\Inc -I.\Drivers\STM32L4xx_HAL_Driver\Inc\Legacy -I.\Drivers\CMSIS\Device\ST\STM32L4xx\Include -I.\Drivers\CMSIS\Include -I.\.cmsis\include -I.\MDK-ARM\RTE\_STM32L475 -I.\.eide\deps
LDFLAGS := --cpu Cortex-M4.fp --library_type=microlib --scatter "c:/XYZ/CubeMX/Project/ZHIHU/STM32L475/STM32L475/build/STM32L475/STM32L475.sct" --strict --summary_stderr --info summarysizes --map --xref --callgraph --symbols --info sizes --info totals --info unused --info veneers --list .\build\STM32L475\STM32L475.map
LDLIBS := 