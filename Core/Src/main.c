/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "ad7730.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t tx_buffer_3[BUFFER_SIZE_SPI3] = {READ_CONFIG,0,0x21,0xB0,0,0x04,0xB0,0x10,
										 0,0,0,0,0,0,0,0,
										 0,0,0,0,0,0,0,0,
										 0,0,0,0,0,0,0,0,
										 0,0,0,0,0,0,0,0,
										 0,0,0,0,0,0,0,0,
										 0,0,0,0,0,0,0,0,
										 0,0,0,0,0,0,0,0};

/* RX_BUFFER FORMAT
 * shadow_buffer[0:slave_state, 1:update_mode_params, 2-3:mode_params{w/o channel}, 4:update_filter_params, 5-7:filter_params]
 */
uint8_t rx_buffer_3[BUFFER_SIZE_SPI3] = {READ_CONFIG,0,0x21,0xB0,0,0x04,0xB0,0x10,
										 0,0,0,0,0,0,0,0,
										 0,0,0,0,0,0,0,0,
										 0,0,0,0,0,0,0,0,
										 0,0,0,0,0,0,0,0,
										 0,0,0,0,0,0,0,0,
										 0,0,0,0,0,0,0,0,
										 0,0,0,0,0,0,0,0};

//uint8_t rx_buffer_3[BUFFER_SIZE_SPI3] = {MONITOR,0x01,0x41,0x80,0x01,0x80,0x00,0x10,
//										 0,0,0,0,0,0,0,0,
//										 0,0,0,0,0,0,0,0,
//										 0,0,0,0,0,0,0,0,
//										 0,0,0,0,0,0,0,0,
//										 0,0,0,0,0,0,0,0,
//										 0,0,0,0,0,0,0,0,
//										 0,0,0,0,0,0,0,0};

/* SHADOW_BUFFER FORMAT
 * shadow_buffer[0...47:channel_1-16, 63:slave_state]
 */
uint8_t shadow_buffer_3_0[BUFFER_SIZE_SPI3];
uint8_t shadow_buffer_3_1[BUFFER_SIZE_SPI3];

uint8_t tx_buffer_2[BUFFER_SIZE_SPI2];
uint8_t rx_buffer_2[BUFFER_SIZE_SPI2];

/*
 * DEBUG_BUFFER
 */
//uint8_t shadow_buffer_3_2[BUFFER_SIZE_SPI3] = {0,0,1,0,0,1,0,0,
//											   1,0,0,1,0,0,1,0,
//											   0,1,0,0,1,0,0,1,
//											   0,0,1,0,0,1,0,0,
//											   1,0,0,1,0,0,1,0,
//											   0,1,0,0,1,0,0,1,
//											   0,0,1,0,0,1,0,0,
//											   1,0,0,1,0,0,0,0};

uint8_t shadow_buffer_3_2[BUFFER_SIZE_SPI3] = {0xDE,0xAD,0xBE,0xEF,0xDE,0xAD,0xBE,0xEF,
											   0xDE,0xAD,0xBE,0xEF,0xDE,0xAD,0xBE,0xEF,
											   0xDE,0xAD,0xBE,0xEF,0xDE,0xAD,0xBE,0xEF,
											   0xDE,0xAD,0xBE,0xEF,0xDE,0xAD,0xBE,0xEF,
											   0xDE,0xAD,0xBE,0xEF,0xDE,0xAD,0xBE,0xEF,
											   0xDE,0xAD,0xBE,0xEF,0xDE,0xAD,0xBE,0xEF,
											   0xDE,0xAD,0xBE,0xEF,0xDE,0xAD,0xBE,0xEF,
											   0xDE,0xAD,0xBE,0xEF,0xDE,0xAD,0xBE,0xEF,};


//Controller_State slave_state = CHECKING_SPI2;
Controller_State slave_state = READ_CONFIG;
uint8_t active_buffer = 0;
uint8_t buffer_updated = 0;
uint8_t conv_cmd = 0;

// Parameters
uint8_t AD7730_REGISTER_SIZE[8] = {1, 3, 2, 3, 1, 3, 3, 3}; //IO, DATA, MODE, FILTER, DAC, OFFSET, GAIN, TEST
uint8_t mode_register[2] = {0x51, 0xB4}; //01010001, 10110100 : modebits(3)| B/U | D_enable | D_value(2) | word length for data register, HIREF |0| input range(2) | MCLKdisable | burn out | Channel selection(2)
//single conversion mode | unipolar | AIN2 set as input | 00 | 24bit, ref=5v |0| Input range: 0to80mV | MCLKenable | burnout on | AIN1+ & AIN1-
uint8_t filter_register[3] = {0x80, 0x03, 0x10}; //default
uint8_t dac_register[TRANSDUCER_NUMBER] = {0x20, 0x20, 0x20}; //default
//uint8_t filter_register[3] = {0x80, 0x00, 0x10}; //SF value = 2048 Base sample rate of 50 Hz, SKIP OFF | FAST OFF, CHOP ON
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static inline void copy_buffer(uint8_t dest_buffer[], uint8_t dest_size, uint8_t src_buffer[], uint8_t src_size) {
  for (uint8_t idx = 0; idx < src_size; idx++) {
    dest_buffer[idx] = src_buffer[idx];
  }
}

static inline void set_state(uint8_t command_idx) {
  switch (command_idx) {
    case 0xFD: //11111101, 253
      slave_state = MONITOR;
      break;

    case 0xFC: //11111100, 252
      slave_state = PROVIDE_DATA;
      break;

    case 0xFB: //11111011, 251
      slave_state = CHECKING_SPI2;
      break;

    case 0xFA: //11111010, 250
      slave_state = SETTING;
      break;

    default:
      slave_state = READ_CONFIG;
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {

  if (hspi->Instance == SPI3) {
//    switch (active_buffer) {
//      case 0:
//        shadow_buffer_3_1[63] = slave_state; //3_1
//        HAL_SPI_TransmitReceive_DMA(&hspi3, shadow_buffer_3_1, rx_buffer_3, BUFFER_SIZE_SPI3);
////        HAL_SPI_TransmitReceive_DMA(&hspi3, shadow_buffer_3_2, rx_buffer_3, BUFFER_SIZE_SPI3);
//        buffer_updated = 1;
//        break;
//
//      case 1:
//        shadow_buffer_3_0[63] = slave_state; //3_0
//        HAL_SPI_TransmitReceive_DMA(&hspi3, shadow_buffer_3_0, rx_buffer_3, BUFFER_SIZE_SPI3);
////        HAL_SPI_TransmitReceive_DMA(&hspi3, shadow_buffer_3_2, rx_buffer_3, BUFFER_SIZE_SPI3);
//        buffer_updated = 1;
//        break;
//    }
    tx_buffer_3[63] = slave_state;
    HAL_SPI_TransmitReceive_DMA(&hspi3, tx_buffer_3, rx_buffer_3, BUFFER_SIZE_SPI3);

    set_state(rx_buffer_3[0]);
    if (rx_buffer_3[1] != 0)
    {
    	mode_register[0] = rx_buffer_3[2];
    	mode_register[1] = rx_buffer_3[3];
    }
    if (rx_buffer_3[4] != 0)
    {
    	filter_register[0] = rx_buffer_3[5];
    	filter_register[1] = rx_buffer_3[6];
    	filter_register[2] = rx_buffer_3[7];
    }
    if (rx_buffer_3[8] != 0)
	{
		for(uint8_t i = 0; i < TRANSDUCER_NUMBER; i++){
			dac_register[0] = rx_buffer_3[i+8];
		}
	}
  }

}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
//  setbuf(stdout, NULL);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  static struct Transducer_SS_Info slave_infos[5] = { { CS0_Pin, CS0_GPIO_Port }, { CS1_Pin, CS1_GPIO_Port }, { CS2_Pin, CS2_GPIO_Port }, { CS3_Pin, CS3_GPIO_Port }, { CS4_Pin, CS4_GPIO_Port }};
  static struct Transducer_RDY_Info ready_infos[5] = { { RDY0_Pin, RDY0_GPIO_Port }, { RDY1_Pin, RDY1_GPIO_Port }, { RDY2_Pin, RDY2_GPIO_Port }, { RDY3_Pin, RDY3_GPIO_Port }, { RDY4_Pin, RDY4_GPIO_Port }};
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(CS0_GPIO_Port, CS0_Pin, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_SET);
  for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++)
	  ad7730_softreset(dev_idx, slave_infos);
  HAL_SPI_TransmitReceive_DMA(&hspi3, tx_buffer_3, rx_buffer_3, BUFFER_SIZE_SPI3);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  switch (slave_state) {

	       case READ_CONFIG:
//	    	   copy_buffer(tx_buffer_3, BUFFER_SIZE_SPI3, rx_buffer_3, BUFFER_SIZE_SPI3);
//	    	   for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
//				   ad7730_read_register(dev_idx, REG_FILTER_REGISTER, &tx_buffer_3[dev_idx * CONFIGDATA_SIZE + 8], slave_infos);
//			   }
//	    	   for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
//	    		   ad7730_read_register(dev_idx, REG_DAC_REGISTER, &tx_buffer_3[dev_idx*CONFIGDATA_SIZE + 11], slave_infos);
//			   }
//	    	   for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
//	    		   ad7730_read_register(dev_idx, REG_MODE_REGISTER, &tx_buffer_3[dev_idx*CONFIGDATA_SIZE + 12], slave_infos);
//			   }
//	    	   if (buffer_updated == 1) {
//	    		   switch (active_buffer) {
//					 case 0:
//					   copy_buffer(shadow_buffer_3_0, BUFFER_SIZE_SPI3, tx_buffer_3, BUFFER_SIZE_SPI3);
//					   break;
//
//					 case 1:
//					   copy_buffer(shadow_buffer_3_1, BUFFER_SIZE_SPI3, tx_buffer_3, BUFFER_SIZE_SPI3);
//					   break;
//				   }
//				   active_buffer ^= 1;
//				   buffer_updated = 0;
//	    	   }
//	    	   break;

	       case CHECKING_SPI2:

//	       	   copy_buffer(tx_buffer_3, BUFFER_SIZE_SPI3, rx_buffer_3, BUFFER_SIZE_SPI3);
//			   for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
//				   ad7730_write_register(dev_idx, REG_FILTER_REGISTER, filter_register, slave_infos);
//			   }
////			   for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
////				   ad7730_read_register(dev_idx, REG_IO_REGISTER, &tx_buffer_3[dev_idx + 8], slave_infos);
////			   }
//			   for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
//				   ad7730_read_register(dev_idx, REG_FILTER_REGISTER, &tx_buffer_3[dev_idx * CONFIGDATA_SIZE+8], slave_infos);
//			   }
////			   for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {//setting a offset voltage between AIN+ and AIN-
////				 uint8_t DAC_command = 0x00 | 0x00; //+ | 20mV
////				 ad7730_write_register(dev_idx, REG_DAC_REGISTER, DAC_command, slave_infos);
////			   }
//			   for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
//				 ad7730_read_register(dev_idx, REG_DAC_REGISTER, &tx_buffer_3[dev_idx*CONFIGDATA_SIZE + 11], slave_infos);
//			   }
////			   for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) { //internal zero calibration
////				 uint8_t conversion_command[2] = {0x91, 0x80 | CHANNEL_A1};
////				 ad7730_write_register(dev_idx, REG_MODE_REGISTER, conversion_command, slave_infos);
////			   }
////			   for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) { //internal full calibration
////				 uint8_t conversion_command[2] = {0xB1, 0x80 | CHANNEL_A1};
////				 ad7730_write_register(dev_idx, REG_MODE_REGISTER, conversion_command, slave_infos);
////			   }
//			   for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
//				 uint8_t conversion_command[2] = {mode_register[0], mode_register[1] | CHANNEL_A1};
//				 ad7730_write_register(dev_idx, REG_MODE_REGISTER, conversion_command, slave_infos);
//			   }
//			   for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
//				 ad7730_read_register(dev_idx, REG_MODE_REGISTER, &tx_buffer_3[dev_idx*CONFIGDATA_SIZE + 12], slave_infos);
//			   }
//			   for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
//				 uint8_t conversion_command[2] = {mode_register[0], mode_register[1] | CHANNEL_A1};
//				 ad7730_write_register(dev_idx, REG_MODE_REGISTER, conversion_command, slave_infos);
//			   }
//			   //wait for ready
//		       for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
//		         while (HAL_GPIO_ReadPin(ready_infos[dev_idx].rdy_port, ready_infos[dev_idx].rdy_pin) == GPIO_PIN_SET) {
//		          uint8_t dbg = dev_idx;
//		         }
//		         ad7730_read_register(dev_idx, REG_DATA_REGISTER, &tx_buffer_3[dev_idx * CONFIGDATA_SIZE + 16], slave_infos);
//		       }
	       	//TODO Make do while loop
//			  if (buffer_updated == 1) {
//	           switch (active_buffer) {
//	             case 0:
//	               copy_buffer(shadow_buffer_3_0, BUFFER_SIZE_SPI3, tx_buffer_3, BUFFER_SIZE_SPI3);
//	               break;
//
//	             case 1:
//	               copy_buffer(shadow_buffer_3_1, BUFFER_SIZE_SPI3, tx_buffer_3, BUFFER_SIZE_SPI3);
//	               break;
//	           }
//			   active_buffer ^= 1;
//			   buffer_updated = 0;
//			 }
			 break;

	       case SETTING:
//	    	   copy_buffer(tx_buffer_3, BUFFER_SIZE_SPI3, rx_buffer_3, BUFFER_SIZE_SPI3);

			   for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
				   ad7730_write_register(dev_idx, REG_FILTER_REGISTER, filter_register, slave_infos);
			   }
//			   for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
//				   ad7730_read_register(dev_idx, REG_FILTER_REGISTER, &tx_buffer_3[dev_idx * CONFIGDATA_SIZE], slave_infos);
////				   ad7730_read_register(dev_idx, REG_FILTER_REGISTER, &tx_buffer_3[dev_idx * CONFIGDATA_SIZE+8], slave_infos);
//			   }
			   for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {//setting a offset voltage between AIN+ and AIN-
//			     uint8_t DAC_command = 0x20 | 0x04; //- | 10mV
				 ad7730_write_register(dev_idx, REG_DAC_REGISTER, &dac_register[dev_idx], slave_infos);
			   }
//			   for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
//				 ad7730_read_register(dev_idx, REG_DAC_REGISTER, &tx_buffer_3[dev_idx*CONFIGDATA_SIZE + 3], slave_infos);
////				 ad7730_read_register(dev_idx, REG_DAC_REGISTER, &tx_buffer_3[dev_idx*CONFIGDATA_SIZE + 11], slave_infos);
//			   }
			   for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
				 uint8_t conversion_command[2] = {0x81, mode_register[1] | CHANNEL_A1}; //Internal Zero Calibaration
				 ad7730_write_register(dev_idx, REG_MODE_REGISTER, conversion_command, slave_infos);
				 while (HAL_GPIO_ReadPin(ready_infos[dev_idx].rdy_port, ready_infos[dev_idx].rdy_pin) == GPIO_PIN_SET) {
//				   uint8_t dbg = dev_idx;
				 }
			   }
			   for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
				 uint8_t conversion_command[2] = {0xA1, mode_register[1] | CHANNEL_A1}; //Internal Full Calibaration
				 ad7730_write_register(dev_idx, REG_MODE_REGISTER, conversion_command, slave_infos);
				 while (HAL_GPIO_ReadPin(ready_infos[dev_idx].rdy_port, ready_infos[dev_idx].rdy_pin) == GPIO_PIN_SET) {
//				   uint8_t dbg = dev_idx;
				 }
			   }
			   for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
				 uint8_t conversion_command[2] = {mode_register[0], mode_register[1] | CHANNEL_A1};
				 ad7730_write_register(dev_idx, REG_MODE_REGISTER, conversion_command, slave_infos);
				 while (HAL_GPIO_ReadPin(ready_infos[dev_idx].rdy_port, ready_infos[dev_idx].rdy_pin) == GPIO_PIN_SET) {
//				   uint8_t dbg = dev_idx;
				 }
			   }
//			   for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
//				 ad7730_read_register(dev_idx, REG_MODE_REGISTER, &tx_buffer_3[dev_idx*CONFIGDATA_SIZE + 4], slave_infos);
////				 ad7730_read_register(dev_idx, REG_MODE_REGISTER, &tx_buffer_3[dev_idx*CONFIGDATA_SIZE + 12], slave_infos);
//			   }
//			   for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
//				 uint8_t conversion_command[2] = {mode_register[0], mode_register[1] | CHANNEL_A1};
//				 ad7730_write_register(dev_idx, REG_MODE_REGISTER, conversion_command, slave_infos);
//				 while (HAL_GPIO_ReadPin(ready_infos[dev_idx].rdy_port, ready_infos[dev_idx].rdy_pin) == GPIO_PIN_SET) {
//				   uint8_t dbg = dev_idx;
//				 }
//			   }

//			   if (buffer_updated == 1) {
//				   switch (active_buffer) {
//				   	   case 0:
//					   	   copy_buffer(shadow_buffer_3_0, BUFFER_SIZE_SPI3, tx_buffer_3, BUFFER_SIZE_SPI3);
//					   	   break;
//
//				   	   case 1:
//				   		   copy_buffer(shadow_buffer_3_1, BUFFER_SIZE_SPI3, tx_buffer_3, BUFFER_SIZE_SPI3);
//				   		   break;
//				   }
//				   active_buffer ^= 1;
//				   buffer_updated = 0;
//			   }
			   break;
//
	       case PROVIDE_DATA:
//	    	 if(!conv_cmd){
//				 for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
//				   uint8_t conversion_command[2] = {0x31, mode_register[1] | CHANNEL_A1};
//				   ad7730_write_register(dev_idx, REG_MODE_REGISTER, conversion_command, slave_infos);
//				 }
////				 for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
////				   uint8_t conversion_command[2] = {0x31, mode_register[1] | CHANNEL_A2};
////				   ad7730_write_register(dev_idx, REG_MODE_REGISTER, conversion_command, slave_infos);
////				 }
//				 conv_cmd = 1;
//	    	 }
//
//			 //TODO Make do while loop
//			 for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
////	           while (HAL_GPIO_ReadPin(ready_infos[dev_idx].rdy_port, ready_infos[dev_idx].rdy_pin) == GPIO_PIN_SET) {
////	         	  //uint8_t dbg = dev_idx;
////	           }
//			   ad7730_read_register(dev_idx, REG_DATA_REGISTER, &tx_buffer_3[dev_idx * 6], slave_infos);
//			 }
//	           switch (active_buffer) {
//	             case 0:
//	               copy_buffer(shadow_buffer_3_0, BUFFER_SIZE_SPI3, tx_buffer_3, BUFFER_SIZE_SPI3);
//	               break;
//
//	             case 1:
//	               copy_buffer(shadow_buffer_3_1, BUFFER_SIZE_SPI3, tx_buffer_3, BUFFER_SIZE_SPI3);
//	               break;
//	           }
//			   active_buffer ^= 1;
//			   buffer_updated = 0;
	         break;

	       case MONITOR:

	         /*
	          * Reason for using six instead of one big loop:
	          *
	          * To give load cell controllers as much time
	          * as possible for processing the input without waiting
	          * data is send to the controllers one by one.
	          * Doing so the time used for sending data to the
	          * remaining controllers gives the first one time to process the input.
	          *
	          */

//             copy_buffer(tx_buffer_3, BUFFER_SIZE_SPI3, rx_buffer_3, BUFFER_SIZE_SPI3);

//	         for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
//	           ad7730_write_register(dev_idx, REG_FILTER_REGISTER, filter_register, slave_infos);
//	         }
             for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
			   ad7730_read_register(dev_idx, REG_FILTER_REGISTER, &tx_buffer_3[dev_idx * CONFIGDATA_SIZE + 16], slave_infos);
			 }
             for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
			   ad7730_read_register(dev_idx, REG_DAC_REGISTER, &tx_buffer_3[dev_idx*CONFIGDATA_SIZE + 19], slave_infos);
		     }
             for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
			   ad7730_read_register(dev_idx, REG_MODE_REGISTER, &tx_buffer_3[dev_idx*CONFIGDATA_SIZE + 20], slave_infos);
			 }
             if(!conv_cmd){ // To set continuous read mode of a channel
				 for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
				   uint8_t conversion_command[2] = {mode_register[0], mode_register[1] | CHANNEL_A1};
				   ad7730_write_register(dev_idx, REG_MODE_REGISTER, conversion_command, slave_infos);
				   while (HAL_GPIO_ReadPin(ready_infos[dev_idx].rdy_port, ready_infos[dev_idx].rdy_pin) == GPIO_PIN_SET) {
//					  uint8_t dbg = dev_idx;
				   }
				 }
				 conv_cmd = 1;
             }

	         //TODO Make do while loop

//	         for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
//	           uint8_t conversion_command[2] = {mode_register[0], mode_register[1] | CHANNEL_A2};
//	           ad7730_write_register(dev_idx, REG_MODE_REGISTER, conversion_command, slave_infos);
//	         }
//
//			 for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
//			   ad7730_read_register(dev_idx, REG_FILTER_REGISTER, &tx_buffer_3[dev_idx*CONFIGDATA_SIZE], slave_infos);
//			 }
//			 for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
//			   ad7730_read_register(dev_idx, REG_DAC_REGISTER, &tx_buffer_3[dev_idx*CONFIGDATA_SIZE + 3], slave_infos);
//			 }
//		     for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
//		       ad7730_read_register(dev_idx, REG_MODE_REGISTER, &tx_buffer_3[dev_idx*CONFIGDATA_SIZE + 3], slave_infos);
//			 }
	         for (uint8_t dev_idx = 0; dev_idx < TRANSDUCER_NUMBER; dev_idx++) {
//	           while (HAL_GPIO_ReadPin(ready_infos[dev_idx].rdy_port, ready_infos[dev_idx].rdy_pin) == GPIO_PIN_SET) {
//	         	  //uint8_t dbg = dev_idx;
//	           }
	           ad7730_read_register(dev_idx, REG_DATA_REGISTER, &tx_buffer_3[dev_idx * DATA_SIZE], slave_infos);
	         }

	         /*
	          * Switch shadow buffer when new data is available
	          * and set indicator flag for SPI-slave callback
	          */
//	         if (buffer_updated == 1) {
//	           switch (active_buffer) {
//	             case 0:
//	               copy_buffer(shadow_buffer_3_0, BUFFER_SIZE_SPI3, tx_buffer_3, BUFFER_SIZE_SPI3);
//	               break;
//
//	             case 1:
//	               copy_buffer(shadow_buffer_3_1, BUFFER_SIZE_SPI3, tx_buffer_3, BUFFER_SIZE_SPI3);
//	               break;
//	           }
//	           active_buffer ^= 1;
//	           buffer_updated = 0;
//	         }

	         break;

	     }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
  int DataIdx;
  for(DataIdx=0; DataIdx<len; DataIdx++)
  {
    ITM_SendChar(*ptr++);
  }
  return len;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1);
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
