#include "stm32f4xx.h"
#include "BSP_SPI.h"


void SPI3_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;

	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI3);
	
	//Configure SPI1 Pins: SCK, MISO and MOSI
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//Configure NSS Pin
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;		//CS/NSS
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC, GPIO_Pin_1);//不选中（关闭片选）--->低电平选通SD卡

	SPI_I2S_DeInit(SPI3);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,ENABLE);	//复位SPI1
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,DISABLE);//停止复位SPI1
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	//双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//主器件
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;	//8位数据长度
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;   		//这里要注意，一定要配置为上升沿数据有效，因为SD卡为上升沿数据有效
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;		//这里要注意，一定要配置为SPI_CPHA_2Edge（数据捕获于第2个时钟沿），参见SD卡协议要求
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;			//NSS信号由外部管脚管理
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;//SPI速度为低速
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//数据传输的第一个字节为MSB
	SPI_InitStructure.SPI_CRCPolynomial = 7;			//CRC的多项式
	SPI_Init(SPI3,&SPI_InitStructure);
	SPI_Cmd(SPI3,DISABLE);
	SPI_Cmd(SPI3,ENABLE);
	
	
}



//SPI 速度设置函数
//SpeedSet:
//SPI_BaudRatePrescaler_2   2分频   (SPI 36M@sys 72M)
//SPI_BaudRatePrescaler_8   8分频   (SPI 9M@sys 72M)
//SPI_BaudRatePrescaler_16  16分频  (SPI 4.5M@sys 72M)
//SPI_BaudRatePrescaler_256 256分频 (SPI 281.25K@sys 72M)
//#define SPI_BaudRatePrescaler_2         ((uint16_t)0x0000)
//#define SPI_BaudRatePrescaler_4         ((uint16_t)0x0008)
//#define SPI_BaudRatePrescaler_8         ((uint16_t)0x0010)
//#define SPI_BaudRatePrescaler_16        ((uint16_t)0x0018)
//#define SPI_BaudRatePrescaler_32        ((uint16_t)0x0020)
//#define SPI_BaudRatePrescaler_64        ((uint16_t)0x0028)
//#define SPI_BaudRatePrescaler_128       ((uint16_t)0x0030)
//#define SPI_BaudRatePrescaler_256       ((uint16_t)0x0038)
void SPI3_SetSpeed(uint8_t SpeedSet)
{
	
  SPI_InitTypeDef SPI_InitStructure;
 
	
	SPI_I2S_DeInit(SPI3);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,ENABLE);	//复位SPI1
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,DISABLE);//停止复位SPI1
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	//双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//主器件
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;	//8位数据长度
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;   		//这里要注意，一定要配置为上升沿数据有效，因为SD卡为上升沿数据有效
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;		//这里要注意，一定要配置为SPI_CPHA_2Edge（数据捕获于第2个时钟沿），参见SD卡协议要求
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;			//NSS信号由外部管脚管理

	switch (SpeedSet)
	{
		case enum_SPI_SPEED_LOW:			
			SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;//设置到低速模式
			break;
		case enum_SPI_SPEED_HIGH:
			SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;	//设置到高速模式  16原来
			break;	
		default:
			SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;//设置到低速模式
			break;			
	}

	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//数据传输的第一个字节为MSB		
	SPI_InitStructure.SPI_CRCPolynomial = 7;			//CRC的多项式
	SPI_Init(SPI3,&SPI_InitStructure);
	SPI_Cmd(SPI3 , DISABLE);
	SPI_Cmd(SPI3 , ENABLE);
}



void SPI_WriteByte(uint8_t _ucByte)
{
	while(SPI_I2S_GetFlagStatus(SPI3,SPI_I2S_FLAG_TXE )==RESET);	//等待数据发送寄存器清空
	SPI_I2S_SendData(SPI3 , _ucByte);								//通过SPI发送出去一个字节数据
	while(SPI_I2S_GetFlagStatus(SPI3 , SPI_I2S_FLAG_RXNE )==RESET);	//等待接收到一个数据（接收到一个数据就相当于发送一个数据完毕）
	SPI_I2S_ReceiveData(SPI3);										//返回接收到的数据
}



uint8_t SPI_ReadByte(void)
{
	uint8_t ch;
	
	
	while(SPI_I2S_GetFlagStatus(SPI3,SPI_I2S_FLAG_TXE )==RESET);
	SPI_I2S_SendData(SPI3 , 0xFF);
	while(SPI_I2S_GetFlagStatus(SPI3,SPI_I2S_FLAG_RXNE )==RESET);
	ch = SPI_I2S_ReceiveData(SPI3);
	return (ch);
}



uint8_t SPI_ReadWriteByte(uint8_t _ucByte)
{  
	uint8_t ch;
	
	
	while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET);		// 等待发送缓冲区空
	SPI_I2S_SendData(SPI3, _ucByte);
	while(SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET);		// 等待数据接收完毕
	ch = SPI_I2S_ReceiveData(SPI3);
	return (ch);
}


