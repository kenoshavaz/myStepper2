// Lab    : Stepper Motor Control
// Project: myStepper2.c
// Name   : Kenosha Vaz
// Date   : 15 March 2018

#include <stdio.h>
#include <stdint.h>
#include <ctype.h>

#include "common.h"

static TIM_HandleTypeDef tim15;

static volatile uint32_t stpCount,dlySpd,increment,current,count;
static volatile uint8_t phase;

/* Initialise the GPIO pins */

ParserReturnVal_t CmdStpInit(int action)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  if(action!=CMD_INTERACTIVE) return CmdReturnOk;

  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 0;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 0;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = 1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  

  /* /\* Initialize DAC *\/ */
  
  /* __HAL_RCC_DAC1_CLK_ENABLE(); */

  /* hdac.Instance=DAC1; */

  /* rc = HAL_DAC_Init(&hdac); */

  /* if(rc != HAL_OK) { */
  /*   printf("Unable to initialize DAC, rc=%lu\n",rc); */
  /*   return CmdReturnOk; */
  /* } */

  /* DacConfig.DAC_Trigger = DAC_TRIGGER_NONE; */
  /* DacConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE; */
  /* rc = HAL_DAC_ConfigChannel(&hdac, &DacConfig,DAC_CHANNEL_1); */

  /* if(rc != HAL_OK) { */
  /*   printf("Unable to configure DAC channel 1, rc=%lu\n",rc); */
  /*   return CmdReturnOk; */
  /* } */

  /* /\* Enable the output *\/  */

  /* __HAL_DAC_ENABLE(&hdac,DAC_CHANNEL_1); */
    
  /*Initialise the Timer Delay*/
  
  __HAL_RCC_TIM15_CLK_ENABLE();

  tim15.Instance = TIM15;
  tim15.Init.Prescaler     = HAL_RCC_GetPCLK2Freq() / 1000000 - 1;
  tim15.Init.CounterMode   = TIM_COUNTERMODE_UP;
  tim15.Init.Period        = 1000;
  tim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  tim15.Init.RepetitionCounter = 0;

  HAL_TIM_Base_Init(&tim15);

  HAL_NVIC_SetPriority(TIM15_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(TIM15_IRQn);

  HAL_TIM_Base_Start(&tim15);

  TIM15->DIER |= 0x01;

  /* Configure Output */

  TIM_OC_InitTypeDef sConfig;

  sConfig.OCMode       = TIM_OCMODE_PWM1;
  sConfig.Pulse        = 1000;
  sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfig.OCNPolarity  = TIM_OCNPOLARITY_LOW;
  sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  HAL_TIM_PWM_ConfigChannel(&tim15,&sConfig, TIM_CHANNEL_1);
    
  return CmdReturnOk;
}
ADD_CMD("stepperinit",CmdStpInit,"               Initialise Stepper Motor");

/* IRQ Handler to reset Watchdog Timer from stopping Program Run */

void TIM15_IRQHandler(){

  TIM15->SR &= ~0x01;
  
  switch(phase){
  case 1:
    if(current>=dlySpd){
      current=current-increment;
      count++;
      TIM15->ARR=current;
      TIM15->CCR1=current/2;
    }else{
      phase++;
    }
    break;
  case 2:
    if(stpCount<=count){
      phase++;
    }
    break;
  case 3:
    if(count){
      current=current+increment;
      count--;
      TIM15->ARR=current;
      TIM15->CCR1=current/2;
    }else{
      phase=0;
    }
    break;
  }
  
  if(stpCount){
    stpCount--;
    if(stpCount==0){
       HAL_TIM_PWM_Stop(&tim15,TIM_CHANNEL_1); /* Stop PWM Output */
    }
  }
}

ParserReturnVal_t CmdStpEnable(int action)
{
  if(action!=CMD_INTERACTIVE) return CmdReturnOk;

  uint16_t rc;
  int16_t pin;

  rc=fetch_int16_arg(&pin);
 
  if(rc){
    printf("\nMust Enter Value!\n\n");
    return CmdReturnBadParameter1;
  }

  if(pin==1){
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3|GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
    
  }else if(pin==0){
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3|GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);

  }else{
    printf("\nPlease Enter 1 / 0 to Toggle pin\n");
  }

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  return CmdReturnOk;
}
ADD_CMD("stepenable",CmdStpEnable,"<1|0>           Enable Stepper Motor");

ParserReturnVal_t CmdStepper(int action)
{
  if(action!=CMD_INTERACTIVE) return CmdReturnOk;

  uint32_t rc, delay;
  int32_t step;

  rc=fetch_int32_arg(&step);
  if(rc){
    printf("\nERROR: No value for steps!\n");
    return CmdReturnBadParameter1;
  }  

  rc=fetch_uint32_arg(&delay);
  if(rc<0){
    printf("\nERROR: Time delay must be positive!");
    return CmdReturnBadParameter1;
  }

  if(step<0){
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
	step=-step;
  }else{
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
  }
  
  stpCount=step;

  phase=0;

  /* Re-initialise Timer Delay Period */

  tim15.Init.Period        = delay;
  HAL_TIM_Base_Init(&tim15);
    
    
  /* Configure Output */

  TIM_OC_InitTypeDef sConfig;

  sConfig.OCMode       = TIM_OCMODE_PWM1;
  sConfig.Pulse        = delay/2;
  sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfig.OCNPolarity  = TIM_OCNPOLARITY_LOW;
  sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  HAL_TIM_PWM_ConfigChannel(&tim15,&sConfig, TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&tim15,TIM_CHANNEL_1); /* Start PWM Output */

  return CmdReturnOk;
}
ADD_CMD("step",CmdStepper,"<step> <delay>  Control Stepper Motor");

ParserReturnVal_t CmdStp2(int action)
{
  if(action!=CMD_INTERACTIVE) return CmdReturnOk;
  uint32_t rc, delay, excel;
  int32_t step;

  rc=fetch_int32_arg(&step);
  if(rc){
    printf("\nERROR: No value for steps!\n");
    return CmdReturnBadParameter1;
  }  

  rc=fetch_uint32_arg(&delay);
  if(rc<0){
    printf("\nERROR: Time delay must be positive!");
    return CmdReturnBadParameter1;
  }

    rc=fetch_uint32_arg(&excel);
  if(rc<0){
    printf("\nERROR: Accelaration must be positive!");
    return CmdReturnBadParameter1;
  }

  if(step<0){
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
	step=-step;
  }else{
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
  }
  
  stpCount=step;

  phase=1;

  increment=excel;

  dlySpd=delay;

  current=1000;

  count=0;

  /* Re-initialise Timer Delay Period */

  tim15.Init.Period        = 1000;
  HAL_TIM_Base_Init(&tim15);
    
    
  /* Configure Output */

  TIM_OC_InitTypeDef sConfig;

  sConfig.OCMode       = TIM_OCMODE_PWM1;
  sConfig.Pulse        = 500;
  sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfig.OCNPolarity  = TIM_OCNPOLARITY_LOW;
  sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
  sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  HAL_TIM_PWM_ConfigChannel(&tim15,&sConfig, TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&tim15,TIM_CHANNEL_1); /* Start PWM Output */

  return CmdReturnOk;
}
ADD_CMD("step2",CmdStp2,"<STP> <DLY> <XL>Control Stepper Motor");
