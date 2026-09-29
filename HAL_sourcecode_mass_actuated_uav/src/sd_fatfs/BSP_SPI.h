
#ifndef __BSP_SPI_H__
#define __BSP_SPI_H__



#define enum_SPI_SPEED_LOW		(0)
#define enum_SPI_SPEED_HIGH		(1)


void SPI3_Configuration(void);
void SPI3_SetSpeed(uint8_t SpeedSet);
void SPI_WriteByte(uint8_t _ucByte);
uint8_t SPI_ReadByte(void);
uint8_t SPI_ReadWriteByte(uint8_t _ucByte);


#endif


