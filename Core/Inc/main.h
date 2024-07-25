/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32l4xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define UP_Pin GPIO_PIN_0
#define UP_GPIO_Port GPIOB
#define DOWN_Pin GPIO_PIN_1
#define DOWN_GPIO_Port GPIOB
#define MODE1_Pin GPIO_PIN_2
#define MODE1_GPIO_Port GPIOB
#define Backlight_Pin GPIO_PIN_14
#define Backlight_GPIO_Port GPIOD
#define SpiReset_Pin GPIO_PIN_15
#define SpiReset_GPIO_Port GPIOD
#define UP_LED_Pin GPIO_PIN_6
#define UP_LED_GPIO_Port GPIOC
#define DWN_LED_Pin GPIO_PIN_7
#define DWN_LED_GPIO_Port GPIOC
#define MODE1_LED_Pin GPIO_PIN_8
#define MODE1_LED_GPIO_Port GPIOC
#define MODE2_LED_Pin GPIO_PIN_9
#define MODE2_LED_GPIO_Port GPIOC
#define HSD_1_Pin GPIO_PIN_0
#define HSD_1_GPIO_Port GPIOD
#define HSD_2_Pin GPIO_PIN_1
#define HSD_2_GPIO_Port GPIOD
#define HSD_3_Pin GPIO_PIN_2
#define HSD_3_GPIO_Port GPIOD
#define HSD_4_Pin GPIO_PIN_3
#define HSD_4_GPIO_Port GPIOD
#define LSD_1_Pin GPIO_PIN_4
#define LSD_1_GPIO_Port GPIOD
#define LSD_2_Pin GPIO_PIN_5
#define LSD_2_GPIO_Port GPIOD
#define LSD_3_Pin GPIO_PIN_6
#define LSD_3_GPIO_Port GPIOD
#define LSD_4_Pin GPIO_PIN_7
#define LSD_4_GPIO_Port GPIOD
#define MODE2_Pin GPIO_PIN_5
#define MODE2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
enum SeqDecode {
				NONE = 0,
				STEP = 1,
				TIME = 2,
				DIGITALOUT = 3,
				ANALOGOUT = 4,
				SEQHEADER = 5,
				SEQCYCLECOUNT = 6,
				COMMENT = 7,
			};

enum Sequencerstate
{
	DISABLED = 0,
	ENABLED = 1,
	STEPTIMERUNNING = 2,
	NEXTSTEPSETUP = 3,
};

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
