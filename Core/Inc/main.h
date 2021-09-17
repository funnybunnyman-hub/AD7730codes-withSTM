/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
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
#define DEV0_RDY_Pin GPIO_PIN_14
#define DEV0_RDY_GPIO_Port GPIOC
#define RESET_Pin GPIO_PIN_3
#define RESET_GPIO_Port GPIOA
#define RDY0_Pin GPIO_PIN_0
#define RDY0_GPIO_Port GPIOB
#define RDY1_Pin GPIO_PIN_1
#define RDY1_GPIO_Port GPIOB
#define RDY2_Pin GPIO_PIN_2
#define RDY2_GPIO_Port GPIOB
#define RDY3_Pin GPIO_PIN_10
#define RDY3_GPIO_Port GPIOB
#define RDY4_Pin GPIO_PIN_11
#define RDY4_GPIO_Port GPIOB
#define CS0_Pin GPIO_PIN_6
#define CS0_GPIO_Port GPIOC
#define CS1_Pin GPIO_PIN_7
#define CS1_GPIO_Port GPIOC
#define CS2_Pin GPIO_PIN_8
#define CS2_GPIO_Port GPIOC
#define CS3_Pin GPIO_PIN_9
#define CS3_GPIO_Port GPIOC
#define CS4_Pin GPIO_PIN_8
#define CS4_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define BUFFER_SIZE_SPI3 64
#define BUFFER_SIZE_SPI2 3
#define TRANSDUCER_NUMBER 3
#define CONFIGDATA_SIZE 8
#define DATA_SIZE 3


typedef enum {
  MONITOR = 0x01,
  READ_CONFIG = 0x02,
  PROVIDE_DATA = 0x03,
  CHECKING_SPI2 = 0x04,
  SETTING = 0x05
} Controller_State;

struct Transducer_SS_Info {
  uint16_t ss_pin;
  GPIO_TypeDef* ss_port;
};

struct Transducer_RDY_Info {
  uint16_t rdy_pin;
  GPIO_TypeDef* rdy_port;
};

struct Transducer_COM_Infos {
  struct Transducer_SS_Info slave_selects[TRANSDUCER_NUMBER];
  struct Transducer_RDY_Info ready_pins[TRANSDUCER_NUMBER];
};

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
