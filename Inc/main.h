/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_ll_rcc.h"
#include "stm32f3xx_ll_bus.h"
#include "stm32f3xx_ll_system.h"
#include "stm32f3xx_ll_exti.h"
#include "stm32f3xx_ll_cortex.h"
#include "stm32f3xx_ll_utils.h"
#include "stm32f3xx_ll_pwr.h"
#include "stm32f3xx_ll_dma.h"
#include "stm32f3xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */
#define LED_Pin LL_GPIO_PIN_3
#define LED_GPIO_Port GPIOB
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif
/*Enables clock for GPIO port B*/
#define CLOCK_ENABLE			*((volatile uint32_t *) (uint32_t)(0x40021000 + 0x00000014U)) |= (uint32_t)(1 << 18);
#define	GPIOA_BASE_ADDR			(uint32_t)(0x48000000U)

//GPIOB peripheral base address
#define	GPIOB_BASE_ADDR			(uint32_t)(0x48000400U)
//MODER register
#define	GPIOB_MODER_REG			*((volatile uint32_t*) ((uint32_t)GPIOB_BASE_ADDR)) &= ~(uint32_t) (0x3 << 6);
//Set mode for pin 3
#define GPIOB_PIN_3				*((volatile uint32_t*) ((uint32_t) GPIOB_BASE_ADDR)) |= (uint32_t) (1 << 6);
//OTYPER register
#define	GPIOB_OTYPER_REG		*((volatile uint32_t*) ((uint32_t) (GPIOB_BASE_ADDR + 0x04U))) &= ~(1 << 3);
//OSPEEDER register
#define GPIOB_OSPEEDER_REG		*((volatile uint32_t*) ((uint32_t) (GPIOB_BASE_ADDR + 0x08U))) &= ~(0x3 << 6);
//PUPDR register
#define GPIOB_PUPDR_REG			*((volatile uint32_t*) ((uint32_t) (GPIOB_BASE_ADDR + 0x0CU))) |= (1 << 12);
//IDR register
#define GPIOB_NO_PULL           *((volatile uint32_t*) ((uint32_t) (GPIOB_BASE_ADDR + 0x0CU))) &= ~(0x3 << 6);

#define GPIOB_IDR_REG			*(uint32_t *)(GPIOB_BASE_ADDR + 0x10U)
//ODR register
#define GPIOB_ODR_REG			*(uint32_t *)(GPIOB_BASE_ADDR + 0x14U)
//BSRR register
#define GPIOB_BSRR_REG			*(uint32_t *)(GPIOB_BASE_ADDR + 0x18U)
//BRR register
#define GPIOB_BRR_REG			*(uint32_t *)(GPIOB_BASE_ADDR + 0x28U)



//RCC base address
#define	RCC_BASE_ADDR			(uint32_t)(0x40021000U)
//AHBEN register
#define	RCC_AHBENR_REG			*((volatile uint32_t *) (uint32_t)(RCC_BASE_ADDR + 0x00000014U))
#define LED_ON					*((volatile uint32_t *)((uint32_t)(0x48000400 + 0x18U))) |= (1 << 3)	//GPIOB pin 3
#define LED_OFF					*((volatile uint32_t *)((uint32_t)0x48000400 + 0x28U)) |= (1 << 3)	//GPIOB pin 3
// Comment
#define BUTTON (*((volatile uint32_t *)((uint32_t)(0x48000400 + 0x10U))) & (1 << 6))

typedef enum EDGE_TYPE EDGE_TYPE;

enum EDGE_TYPE{
	NONE=0,
	RISE=1,
	FALL=2
};

EDGE_TYPE edgeDetect(uint8_t pin_state, uint8_t samples);
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
