#ifndef AD7730_H_
#define AD7730_H_

#include "stm32f1xx_hal.h"
#include "main.h"
#include "spi.h"
#include "gpio.h"

/*

#define READ_ONLY 0xFF
//Communication Register Values
#define CR_SINGLE_WRITE 0x00
#define CR_SINGLE_READ 0x10
#define CR_CONTINUOUS_READ_START 0x20
#define CR_CONTINUOUS_READ_STOP 0x30

#define CR_COMMUNICATION_REGISTER 0x00 //Write only
#define CR_STATUS_REGISTER 0x00 //Read only
#define CR_DATA_REGISTER 0x01
#define CR_MODE_REGISTER 0x02
#define CR_FILTER_REGISTER 0x03
#define CR_DAC_REGISTER 0x04
#define CR_OFFSET_REGISTER 0x05
#define CR_GAIN_REGISTER 0x06
#define CR_TEST_REGISTER 0x07

//Mode Register Values
#define MR1_MODE_IDLE 0x00
#define MR1_MODE_CONTINUOUS 0x20 //Standard Operation
#define MR1_MODE_SINGLE 0x40
#define MR1_MODE_STANDBY 0x60
#define MR1_MODE_INTERNAL_ZERO_CALIBRATION 0x80
#define MR1_MODE_INTERNAL_FULL_CALIBRATION 0xA0
#define MR1_MODE_SYSTEM_ZERO_CALIBRATION 0xC0
#define MR1_MODE_SYSTEM_FULL_CALIBRATION 0xE0
#define MR1_BU_BIPOLAR 0x00 //+- voltage defined by MR0_RANGE
#define MR1_BU_UNIPOLAR 0x10 //0 to voltage deifined by MRO_RANGE
#define MR1_WL_24_BIT 0x01
#define MR1_WL_16_BIT 0x00


#define MR0_CHANNEL_1 0x00
#define MR0_CHANNEL_2 0x01
#define MR0_CHANNEL_SHORT_1 0x02 //Used for internal noise check
#define MR0_CHANNEL_NEGATIVE_1_2 0x03 //Unknown use
#define MRO_BURNOUT_ON 0x04 //Advanced, to check if loadcell is burnt out

//Filter Register Values
#define FR2_SINC_AVERAGING_2048 0x80  //Base sample rate of 50 Hz
#define FR2_SINC_AVERAGING_1024 0x40  //Base sample rate of 100 Hz
#define FR2_SINC_AVERAGING_512 0x20   //Base sample rate of 200 Hz
#define FR2_SINC_AVERAGING_256 0x10   //Base sample rate of 400 Hz

#define FR1_SKIP_ON 0x02 //the FIR filter on the part is bypassed
#define FR1_SKIP_OFF 0x00
#define FR1_FAST_ON 0x01 //FIR is replaced with moving average on large step, sinc filter averages are used to compensate
#define FR1_FAST_OFF 0x00

#define FR0_CHOP_ON 0x10 //When the chop mode is enabled, the part is effectively chopped at its input and output to remove all offset and offset drift errors on the part.
#define FR0_CHOP_OFF 0x00 //Increases sample rate by x3

//DAC Register Values
#define DACR_OFFSET_SIGN_POSITIVE 0x00
#define DACR_OFFSET_SIGN_NEGATIVE 0x20
#define DACR_OFFSET_40MV 0x10
#define DACR_OFFSET_20MV 0x08
#define DACR_OFFSET_10MV 0x04
#define DACR_OFFSET_5MV 0x02
#define DACR_OFFSET_2P5MV 0x01
#define DACR_OFFSET_NONE 0x00

//current settings
#define CURRENT_MODE_1_SETTINGS (MR1_BU_UNIPOLAR | MR1_WL_24_BIT)
#define CURRENT_MODE_0_SETTINGS (MR0_HIREF_5V | MR0_RANGE_10MV | MR0_CHANNEL_1)*/

#define AD7730_USING_DATA_LENGTH 3

typedef enum {
SINC_2048 = 0x80,  //50 Hz
SINC_1024 = 0x40,  //100 Hz
SINC_512 = 0x20,   //200 Hz
SINC_256 = 0x10,   //400 Hz
SKIP_FIR = 0x02,
FAST_FIR = 0x01
} AD7730_FilterSettingTypeDef;

typedef enum {
HIREF_5V = 0x80,
HIREF_2P5V = 0x00,
} AD7730_ReferenceTypeDef;

typedef enum {
RANGE_10MV = 0x00,
RANGE_20MV = 0x01,
RANGE_40MV = 0x02,
RANGE_80MV = 0x03
} AD7730_InputRangeTypeDef;

typedef enum {
REG_IO_REGISTER = 0x00,
REG_DATA_REGISTER = 0x01,
REG_MODE_REGISTER = 0x02,
REG_FILTER_REGISTER = 0x03,
REG_DAC_REGISTER = 0x04,
REG_OFFSET_REGISTER = 0x05,
REG_GAIN_REGISTER = 0x06,
REG_TEST_REGISTER = 0x07
} AD7730_RegisterTypeDef;

typedef enum {
OP_WRITE = 0x00,
OP_READ = 0x10,
OP_CONTINUOUS_READ_START = 0x20,
OP_CONTINUOUS_READ_STOP = 0x30
} AD7730_CommunicationTypeDef;

typedef enum {
MODE_IDLE = 0x00,
MODE_CONTINUOUS = 0x20,
MODE_SINGLE = 0x40,
MODE_STANDBY = 0x60,
MODE_INTERNAL_ZERO_CALIBRATION = 0x80,
MODE_INTERNAL_FULL_CALIBRATION = 0xA0,
MODE_SYSTEM_ZERO_CALIBRATION = 0xC0,
MODE_SYSTEM_FULL_CALIBRATION = 0xE0,
MODE_BIPOLAR = 0x00,
MODE_UNIPOLAR = 0x10,
MODE_WORD_LENGTH_24_BIT = 0x01,
MODE_WORD_LENGTH_16_BIT = 0x00
} AD7730_ModeConfigurationTypeDef;

typedef enum {
DATA_WIDTH_2B = 0x02,
DATA_WIDTH_3B = 0x03,
} AD7730_DataWidthTypeDef;

typedef enum {
CHANNEL_A1 = 0x00,
CHANNEL_A2 = 0x01
} AD7730_ChannelIndexTypeDef;

extern uint8_t AD7730_REGISTER_SIZE[8];
extern uint8_t filter_register[3];
extern uint8_t mode_register[2];

//void ad7730_setup(uint8_t device, struct Transducer_SS_Info device_infos[]);
//void ad7730_softreset(uint8_t device, struct Transducer_SS_Info device_infos[]);
//void ad7730_system_zero_scale_calibration(uint8_t device, struct Transducer_SS_Info device_infos[]);
//void ad7730_internal_zero_scale_calibration(uint8_t device, struct Transducer_SS_Info device_infos[]);
//void ad7730_set_communication_mode(uint8_t device, AD7730_CommunicationTypeDef com_type, AD7730_RegisterTypeDef reg_type, struct Transducer_SS_Info device_infos[]);
//void ad7730_read_register(uint8_t device, AD7730_RegisterTypeDef reg, uint8_t data[], struct Transducer_SS_Info device_infos[]);
//void ad7730_write_register(uint8_t device, AD7730_RegisterTypeDef reg, uint8_t data[], struct Transducer_SS_Info device_infos[]);
//void ad7730_set_filter(uint8_t device, struct Transducer_SS_Info device_infos[]);

#endif /* AD7730_H_ */
