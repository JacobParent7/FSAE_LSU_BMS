/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f2xx_hal.h"

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
#define TEST_LED_Pin GPIO_PIN_2
#define TEST_LED_GPIO_Port GPIOC
#define CHARGE_EN_LV_Pin GPIO_PIN_4
#define CHARGE_EN_LV_GPIO_Port GPIOC
#define DISCHARGE_EN_LV_Pin GPIO_PIN_5
#define DISCHARGE_EN_LV_GPIO_Port GPIOC
#define CHARGE_SAFE_LV_Pin GPIO_PIN_0
#define CHARGE_SAFE_LV_GPIO_Port GPIOB
#define CHARGE_PWR_LV_ISO_Pin GPIO_PIN_1
#define CHARGE_PWR_LV_ISO_GPIO_Port GPIOB
#define READY_PWR_LV_ISO_Pin GPIO_PIN_2
#define READY_PWR_LV_ISO_GPIO_Port GPIOB
#define FAN_EN_LV_Pin GPIO_PIN_10
#define FAN_EN_LV_GPIO_Port GPIOB
#define BQ_NFAULT_Pin GPIO_PIN_11
#define BQ_NFAULT_GPIO_Port GPIOB
#define BQ_SPI_RDY_Pin GPIO_PIN_14
#define BQ_SPI_RDY_GPIO_Port GPIOB
#define SDIO_DET_Pin GPIO_PIN_15
#define SDIO_DET_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
