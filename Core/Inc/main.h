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
#include "stm32f2xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LiveLed.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define DEVICE_OK             0
#define DEVICE_FAIL           1

#define DEVICE_NAME          "BOOTLOADER"
#define DEVICE_FW            "220713_0828"
#define DEVICE_PCB           "NUCLEO-F207ZG"
#define DEVICE_MNF           "KONVOLUCIO"
#define DEVICE_MNF_SIZE       sizeof(DEVICE_MNF)

#define USB_BUFFER_SIZE     700
#define USB_CMD_LENGTH      35
#define USB_ARG1_LENGTH     35
#define USB_ARG2_LENGTH     35
#define USB_ARG3_LENGTH     35
#define USB_ARG4_LENGTH     513
#define USB_ARG5_LENGTH     35

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MX25_CS_Pin GPIO_PIN_4
#define MX25_CS_GPIO_Port GPIOA
#define LIVE_LED_Pin GPIO_PIN_7
#define LIVE_LED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
