/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
typedef uint32_t u32;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define REP_RATE 1
#define CKTIM 168000000
#define DEADTIME_NS 1000
#define ALIGNMENT_ANGLE 0
#define POLE_PAIR_NUM 14
#define PWM_PRSC 0
#define ICx_FILTER 8
#define PWM_FREQ 10000
#define COUNTER_RESET (ALIGNMENT_ANGLE*4*ENCODER_PPR/360-1)/POLE_PAIR_NUM
#define PWM_PERIOD CKTIM/(2*PWM_FREQ*(PWM_PRSC+1))
#define DEADTIME CKTIM/1000000/2*DEADTIME_NS/1000
#define ENCODER_PPR 2500
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define LCD_CS_Pin GPIO_PIN_15
#define LCD_CS_GPIO_Port GPIOA
#define LCD_DC_Pin GPIO_PIN_11
#define LCD_DC_GPIO_Port GPIOC
#define MT_CS_Pin GPIO_PIN_2
#define MT_CS_GPIO_Port GPIOD
#define LCD_BLK_Pin GPIO_PIN_8
#define LCD_BLK_GPIO_Port GPIOB
#define LCD_RES_Pin GPIO_PIN_9
#define LCD_RES_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */


typedef uint8_t u8;
typedef uint16_t u16;

typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;

#define NB_CONVERSIONS 16
typedef enum {FALSE = 0, TRUE = !FALSE} bool;

#define U8_MAX     ((u8)255)
#define S8_MAX     ((s8)127)
#define S8_MIN     ((s8)-128)
#define U16_MAX    ((u16)65535u)
#define S16_MAX    ((s16)32767)
#define S16_MIN    ((s16)-32768)
#define U32_MAX    ((u32)4294967295uL)
#define S32_MAX    ((s32)2147483647)
#define S32_MIN    ((s32)-2147483648)

#define ABS(x) ((x)>=0?(x):-(x))
#define pi					3.14159265358979f


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
