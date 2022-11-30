#include "Sensor_SPI.h"
#include "spi.h"


uint16_t SPI_AS5048A_ReadData(void)
{
  uint16_t angle_value;
	u16 command,angleValue;
  command = __Read_Angle;
  __AS5048A2_CS_ENABLE;
  HAL_SPI_Transmit(&hspi1 ,(unsigned char *)&command ,1,100);
  __AS5048A2_CS_DISABLE;
  __AS5048A2_CS_ENABLE;
  HAL_SPI_TransmitReceive(&hspi1 ,(unsigned char *)&command ,(unsigned char *)&angleValue ,1 ,100 );
  __AS5048A2_CS_DISABLE;
  angle_value = angleValue & 0x3FFF;
  return angle_value;
}
