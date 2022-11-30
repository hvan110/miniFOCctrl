#ifndef S_SPI_H
#define S_SPI_H

#include "main.h"

#define __AS5048A2_CS_ENABLE  HAL_GPIO_WritePin(MT_CS_GPIO_Port, MT_CS_Pin, GPIO_PIN_RESET)
#define __AS5048A2_CS_DISABLE HAL_GPIO_WritePin(MT_CS_GPIO_Port, MT_CS_Pin, GPIO_PIN_SET)


#define __Read_NOP 0xc000
#define __Read_Clear_Error_Flag 0x4001
#define __Read_Angle 0xffff



uint16_t SPI_AS5048A_ReadData(void);


#endif
