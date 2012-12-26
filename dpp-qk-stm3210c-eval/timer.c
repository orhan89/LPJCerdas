/**
  ******************************************************************************
  * @file    src/timer.c 
  * @author  Ricky Hariady & Eka Rakhman Priandana
  * @version V0.1
  * @date    12/7/2012
  * @brief   Timer Configuration
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  PWM Initialization.
  * @param  None
  * @retval None
  */

void PWMInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_ICInitTypeDef  TIM_ICInitStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  uint16_t PrescalerValue = 0;

  /* TIM5 clocks enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
  
  /* TIM1, GPIOA, GPIOB, GPIOE and AFIO clocks enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1  | RCC_APB2Periph_GPIOA |
			 RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC |
			 RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);

  /* GPIOA Configuration: PE.09 (TIM1 CH1) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOE, &GPIO_InitStructure);

  GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);

  /* GPIOA Configuration: PA.01 (TIM5 CH2) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Prescaler configuration -> CNT_CLK = 562500 Hz */
  PrescalerValue = (uint16_t) (SystemFrequency / 562500) - 1;

  /* Time base configuration (TIM5 & TIM8)*/
  /* Period 17,78 ms */
  TIM_TimeBaseStructure.TIM_Period = 10000;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  /* TIM5 Channel2 : Output Compare Toggle Mode Configuration */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 1000;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  
  TIM_OC2Init(TIM5, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);

  /* TIM5 Channel 1 : Input Capture Configuration */
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0;

  TIM_ICInit(TIM5, &TIM_ICInitStructure);

  /* TIM5 Input trigger configuration: External Trigger (slave mode) connected to TI1 */
  TIM_SelectInputTrigger(TIM5, TIM_TS_TI1FP1);
  TIM_SelectSlaveMode(TIM5, TIM_SlaveMode_Reset);

  /* TIM5 Select the Master Slave Mode */
  TIM_SelectMasterSlaveMode(TIM5, TIM_MasterSlaveMode_Enable);

  /* TIM5 Master Mode selection */
  TIM_SelectOutputTrigger(TIM5, TIM_TRGOSource_OC2Ref);

  /* TIM 8 Master Configuration in Toggle Mode */
  /* PWM Duty cycle determined by TIM_Pulse */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 2800;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM1, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

  /* TIM8 Input trigger configuration: Reset (slave mode) connected to TIM_TS_ITR3 (TIM5) */
  TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Reset);
  TIM_SelectInputTrigger(TIM1, TIM_TS_ITR0);

  //TIM_ARRPreloadConfig(TIM5, ENABLE);
  TIM_ARRPreloadConfig(TIM1, ENABLE);

  /* TIM1 Main PWM Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);

  /* TIM enable counter */
  TIM_Cmd(TIM5, ENABLE);
  TIM_Cmd(TIM1, ENABLE);

  TIM_SetCompare1(TIM1, 3500);

}
