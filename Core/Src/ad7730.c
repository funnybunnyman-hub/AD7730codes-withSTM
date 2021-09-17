#include "ad7730.h"

void ad7730_setup(uint8_t device, struct Transducer_SS_Info device_infos[]) {
}

void ad7730_set_filter(uint8_t device, struct Transducer_SS_Info device_infos[]) {
  //uint8_t filter_register[3] = {0x08,0x43, 0x00}; //Best yet
//  uint8_t filter_register[3] = {0x80,0x00, 0x10}; //Max filter
  uint8_t filter_register[3] = {0x80,0x03, 0x10}; //Max filter and skip
  //uint8_t filter_register[3] = {0x20,0x01, 0x00}; //Conservative
  ad7730_write_register(device, REG_FILTER_REGISTER, filter_register, device_infos);
}

void ad7730_softreset(uint8_t device, struct Transducer_SS_Info device_infos[]) {
  HAL_GPIO_WritePin(device_infos[device].ss_port, device_infos[device].ss_pin, GPIO_PIN_RESET);

  uint8_t command[4] = { 0xFF, 0xFF, 0xFF, 0xFF};
  HAL_SPI_Transmit(&hspi2, command, 4, 10);

  HAL_GPIO_WritePin(device_infos[device].ss_port, device_infos[device].ss_pin, GPIO_PIN_SET);
}

void ad7730_system_zero_scale_calibration(uint8_t device, struct Transducer_SS_Info device_infos[]) {

}

void ad7730_internal_zero_scale_calibration(uint8_t device, struct Transducer_SS_Info device_infos[]) {

}

void ad7730_set_communication_mode(uint8_t device, AD7730_CommunicationTypeDef com_type, AD7730_RegisterTypeDef reg_type, struct Transducer_SS_Info device_infos[]) {

  uint8_t command[1] = { com_type | reg_type };
  HAL_SPI_Transmit(&hspi2, command, 1, 10);

}

void ad7730_read_register(uint8_t device, AD7730_RegisterTypeDef reg, uint8_t data[], struct Transducer_SS_Info device_infos[]) {

  HAL_GPIO_TogglePin(device_infos[device].ss_port, device_infos[device].ss_pin);

  ad7730_set_communication_mode(device, OP_READ, reg, device_infos);

  HAL_SPI_Receive(&hspi2, data, AD7730_REGISTER_SIZE[reg], 10);

  HAL_GPIO_TogglePin(device_infos[device].ss_port, device_infos[device].ss_pin);

}

void ad7730_write_register(uint8_t device, AD7730_RegisterTypeDef reg, uint8_t data[], struct Transducer_SS_Info device_infos[]) {

  HAL_GPIO_TogglePin(device_infos[device].ss_port, device_infos[device].ss_pin);

  ad7730_set_communication_mode(device, OP_WRITE, reg, device_infos);

  HAL_SPI_Transmit(&hspi2, data, AD7730_REGISTER_SIZE[reg], 10);

  HAL_GPIO_TogglePin(device_infos[device].ss_port, device_infos[device].ss_pin);

}
