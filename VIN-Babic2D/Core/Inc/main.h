/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_dma.h"

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
#define JOY_BTN_Pin LL_GPIO_PIN_13
#define JOY_BTN_GPIO_Port GPIOC
#define BTN_ESC_Pin LL_GPIO_PIN_14
#define BTN_ESC_GPIO_Port GPIOC
#define BTN_OK_Pin LL_GPIO_PIN_15
#define BTN_OK_GPIO_Port GPIOC
#define LED_1_Pin LL_GPIO_PIN_3
#define LED_1_GPIO_Port GPIOF
#define LED_2_Pin LL_GPIO_PIN_4
#define LED_2_GPIO_Port GPIOF
#define LED_3_Pin LL_GPIO_PIN_5
#define LED_3_GPIO_Port GPIOF
#define LED_4_Pin LL_GPIO_PIN_0
#define LED_4_GPIO_Port GPIOC
#define LED_5_Pin LL_GPIO_PIN_1
#define LED_5_GPIO_Port GPIOC
#define LED_6_Pin LL_GPIO_PIN_2
#define LED_6_GPIO_Port GPIOC
#define LED_7_Pin LL_GPIO_PIN_3
#define LED_7_GPIO_Port GPIOC
#define LED_0_Pin LL_GPIO_PIN_2
#define LED_0_GPIO_Port GPIOF
#define JOY_X_Pin LL_GPIO_PIN_14
#define JOY_X_GPIO_Port GPIOB
#define JOY_Y_Pin LL_GPIO_PIN_15
#define JOY_Y_GPIO_Port GPIOB
#define BTN_UP_Pin LL_GPIO_PIN_0
#define BTN_UP_GPIO_Port GPIOG
#define BTN_DOWN_Pin LL_GPIO_PIN_1
#define BTN_DOWN_GPIO_Port GPIOG
#define SPI_SCK_Pin LL_GPIO_PIN_2
#define SPI_SCK_GPIO_Port GPIOG
#define SPI_MISO_Pin LL_GPIO_PIN_3
#define SPI_MISO_GPIO_Port GPIOG
#define SPI_MOSI_Pin LL_GPIO_PIN_4
#define SPI_MOSI_GPIO_Port GPIOG
#define BTN_LEFT_Pin LL_GPIO_PIN_6
#define BTN_LEFT_GPIO_Port GPIOG
#define BTN_RIGHT_Pin LL_GPIO_PIN_8
#define BTN_RIGHT_GPIO_Port GPIOG
#define LCD_RST_Pin LL_GPIO_PIN_3
#define LCD_RST_GPIO_Port GPIOD
#define TCH_pending_request_Pin LL_GPIO_PIN_6
#define TCH_pending_request_GPIO_Port GPIOD
#define LCD_BKLT_Pin LL_GPIO_PIN_6
#define LCD_BKLT_GPIO_Port GPIOB
#define USART_3_RX_Pin LL_GPIO_PIN_8
#define USART_3_RX_GPIO_Port GPIOB
#define USART_3_TX_Pin LL_GPIO_PIN_9
#define USART_3_TX_GPIO_Port GPIOB
#define TCH_ChipSelect_Pin LL_GPIO_PIN_1
#define TCH_ChipSelect_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
