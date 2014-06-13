
transmit从joystick读取方向，从ADC读取速度，通过USARTy红外串口发送。
Receive收到方向后改变输出方向，并依据收到的ADC数据通过TIM3以OCtoggle方式改变PWM波占空比。
效果可以通过Receive端的LED4验证。
1、把两个文件夹都放在MDV-STM32F107-basic-examples-3.0(for_ST_FW_3.10)\basic_examples\STM32F10x_StdPeriph_Examples\16-USART\08-IrDA目录下，最好把原文件备份并移除。
2、打开文件后需要手动添加stm32f10x_adc.c和stm32f10x_tim.c。