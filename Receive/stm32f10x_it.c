/**
  ******************************************************************************
  * @file    USART/IrDA/Receive/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.1.0
  * @date    06/19/2009
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and peripherals
  *          interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
//////////////////////////////////////////////////////
#include "stm3210c_eval_lcd.h"
#include "stm32_eval.h"
#include <stdio.h>
void *directionFunc;
extern uint16_t speed;
extern uint8_t funcNum; 
void Forward(void);
void Backward(void);
void Left_turn(void) ;
void Right_turn(void)  ;
void Stop(void);
//////////////////////////////////////////////////////

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup USART_IrDA_Receive
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

////////////////////////////////////////////////////////
void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
	{
		
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update );
		TIM_SetCompare1(TIM3, speed );
		STM_EVAL_LEDOn(LED4);
		switch(funcNum)
		{
			case 1:
				Forward();
				break;
			case 2:
				Backward();
				break;
			case 3:
				Left_turn();
				break;
			case 4:
				Right_turn();
				break;
			default:
				break;
		}
	}
	 if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)
	 {
	 	TIM_ClearITPendingBit(TIM3, TIM_IT_CC1 );
		Stop();
		STM_EVAL_LEDOff(LED4);
	 }
}
void Forward(void)
{
  //printf("Forward , speed = %d\r",speed);
  /*使能驱动模块*/
  GPIO_SetBits(GPIOC, GPIO_Pin_0);
  GPIO_SetBits(GPIOC, GPIO_Pin_1);
  GPIO_SetBits(GPIOA, GPIO_Pin_3);
  GPIO_SetBits(GPIOA, GPIO_Pin_4);
  /*小车前进时，I/O口电平配置*/
  GPIO_ResetBits(GPIOC, GPIO_Pin_3);
  GPIO_SetBits(GPIOC, GPIO_Pin_2);

  GPIO_ResetBits(GPIOA, GPIO_Pin_2);
  GPIO_SetBits(GPIOA, GPIO_Pin_1);
	  
  GPIO_SetBits(GPIOD, GPIO_Pin_9);
  GPIO_ResetBits(GPIOD, GPIO_Pin_10);

  GPIO_SetBits(GPIOD, GPIO_Pin_12);
  GPIO_ResetBits(GPIOD, GPIO_Pin_11);


}

void Backward(void)
{
  //printf("Backward , speed = %d\r",speed);
  /*使能驱动模块*/
  GPIO_SetBits(GPIOC, GPIO_Pin_0);
  GPIO_SetBits(GPIOC, GPIO_Pin_1);
  GPIO_SetBits(GPIOA, GPIO_Pin_3);
  GPIO_SetBits(GPIOA, GPIO_Pin_4);
  /*小车后退时，I/O口电平配置*/
  GPIO_ResetBits(GPIOC, GPIO_Pin_2);
  GPIO_SetBits(GPIOC, GPIO_Pin_3);

  GPIO_ResetBits(GPIOA, GPIO_Pin_1);
  GPIO_SetBits(GPIOA, GPIO_Pin_2);
	  
  GPIO_SetBits(GPIOD, GPIO_Pin_10);
  GPIO_ResetBits(GPIOD, GPIO_Pin_9);

  GPIO_SetBits(GPIOD, GPIO_Pin_11);
  GPIO_ResetBits(GPIOD, GPIO_Pin_12);


}
void Left_turn(void)
{
   //printf("Left_turn , speed = %d\r",speed);
   /*使能驱动模块*/
   GPIO_SetBits(GPIOC, GPIO_Pin_0);
   GPIO_SetBits(GPIOC, GPIO_Pin_1);
   GPIO_SetBits(GPIOA, GPIO_Pin_3);
   GPIO_SetBits(GPIOA, GPIO_Pin_4);

   GPIO_ResetBits(GPIOC, GPIO_Pin_3);
   GPIO_SetBits(GPIOC, GPIO_Pin_2);

   GPIO_ResetBits(GPIOA, GPIO_Pin_2);
   GPIO_SetBits(GPIOA, GPIO_Pin_1);
	  
   GPIO_SetBits(GPIOD, GPIO_Pin_10);
   GPIO_ResetBits(GPIOD, GPIO_Pin_9);

   GPIO_SetBits(GPIOD, GPIO_Pin_11);
   GPIO_ResetBits(GPIOD, GPIO_Pin_12);


}
void Right_turn(void)
{
   
//   printf("Right_turn , speed = %d\r",speed);
   /*使能驱动模块*/
   GPIO_SetBits(GPIOC, GPIO_Pin_0);
   GPIO_SetBits(GPIOC, GPIO_Pin_1);
   GPIO_SetBits(GPIOA, GPIO_Pin_3);
   GPIO_SetBits(GPIOA, GPIO_Pin_4);

   GPIO_ResetBits(GPIOC, GPIO_Pin_2);
   GPIO_SetBits(GPIOC, GPIO_Pin_3);

   GPIO_ResetBits(GPIOA, GPIO_Pin_1);
   GPIO_SetBits(GPIOA, GPIO_Pin_2);
	  
   GPIO_SetBits(GPIOD, GPIO_Pin_9);
   GPIO_ResetBits(GPIOD, GPIO_Pin_10);

   GPIO_SetBits(GPIOD, GPIO_Pin_12);
   GPIO_ResetBits(GPIOD, GPIO_Pin_11);


}
void Stop(void)
{
	GPIO_ResetBits(GPIOC, GPIO_Pin_0);
   	GPIO_ResetBits(GPIOC, GPIO_Pin_1);
   	GPIO_ResetBits(GPIOA, GPIO_Pin_3);
   	GPIO_ResetBits(GPIOA, GPIO_Pin_4);
}
//////////////////////////////////////////////////////

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
