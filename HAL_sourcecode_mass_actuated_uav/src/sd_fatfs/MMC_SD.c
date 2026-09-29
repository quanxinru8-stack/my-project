#include <stdlib.h>
#include <stdio.h>
#include "string.h"
#include "delay.h"
#include "MMC_SD.h"
#include "BSP_SPI.h"
#include "led.h"
#include "usart.h"
#include "time.h"
#include "ff.h"
/********************************************************************************
 * Project : ATKflight飞控固件
 * Brief : SD CARD驱动代码
 * Author : Jiang Chunli
 * Date : 2022/3/28
 * All rights reserved.
********************************************************************************/

u8  SD_Type=0;//SD卡的类型 

//搭载文件系统的全局变量
FIL fil;
FRESULT res;
UINT bww;
FATFS fs;
char name[50];
char startline[50] = "Start writing!\n";

char timestamp[20];
char state_attitude[20];
char set_attitude[20];
char control_value[20];
char data_row[80];

void Delay(uint32_t n)
{
	uint32_t i;
	
	
	for (i = 0 ; i <n; i++)
	{
		__NOP();
		__NOP();
		__NOP();
		__NOP();
		__NOP();
	}
}

//SD卡初始化的时候,需要低速
void SD_SPI_SpeedLow(void)
{
 	//SPI2_SetSpeed(SPI_BaudRatePrescaler_256);//设置到低速模式
	SPI3_SetSpeed(enum_SPI_SPEED_LOW);
}



//SD卡正常工作的时候,可以配置为高速
void SD_SPI_SpeedHigh(void)
{
	//SPI3_SetSpeed(SPI_BaudRatePrescaler_8);//设置到高速模式
	SPI3_SetSpeed(enum_SPI_SPEED_HIGH);
}



//取消选择,释放SPI总线
void SD_DisSelect(void)
{
	GPIO_SetBits(GPIOC,GPIO_Pin_1);
	SPI_WriteByte(0xff);//提供额外的8个时钟
}



//选择sd卡,并且等待卡准备OK
//返回值:0,成功;1,失败;
u8 SD_Select(void)
{
	GPIO_ResetBits(GPIOC,GPIO_Pin_1);
	if(SD_WaitReady()==0)return 0;//等待成功
	SD_DisSelect();
	return 1;//等待失败
}



//等待卡准备好
//返回值:0,准备好了;其他,错误代码
u8 SD_WaitReady(void)
{
	u32 t=0;
	do
	{
		if (SPI_ReadByte()==0XFF)return 0;//OK
		t++;
	}while(t<0XFFF);//等待 
	return 1;
}



//等待SD卡回应
//Response:要得到的回应值
//返回值:0,成功得到了该回应值
//其他,得到回应值失败
u8 SD_GetResponse(u8 Response)
{
	u16 Count=0xFFFF;//等待次数
	while ((SPI_ReadByte()!=Response) && Count) Count--;//等待得到准确的回应(等待得到读取数据令牌0xfe)
	if (Count==0)
		return MSD_RESPONSE_FAILURE;	//得到回应失败
	else 
		return MSD_RESPONSE_NO_ERROR;	//正确回应
}



//从sd卡读取一个数据包的内容
//buf:数据缓存区
//len:要读取的数据长度.
//返回值:0,成功;其他,失败;
u8 SD_RecvData(u8*buf,u16 len)
{
	if (SD_GetResponse(0xFE)) return 1;//等待SD卡发回数据起始令牌0xFE
	while(len--)//开始接收数据
	{
		*buf=SPI_ReadByte();
		buf++;
	}
	//下面是2个伪CRC（dummy CRC）
	SPI_WriteByte(0xFF);
	SPI_WriteByte(0xFF);
	return 0;//读取成功
}



//向sd卡写入一个数据包的内容 512字节
//buf:数据缓存区
//cmd:指令
//返回值:0,成功;其他,失败;
u8 SD_SendBlock(u8*buf,u8 cmd)
{
	u16 t;
	if(SD_WaitReady())return 1;//等待准备失效
	SPI_WriteByte(cmd);
	if(cmd!=0XFD)//不是结束指令
	{
		for(t=0; t<512; t++)SPI_WriteByte(buf[t]); //提高速度,减少函数传参时间
		SPI_WriteByte(0xFF);//忽略crc
		SPI_WriteByte(0xFF);
		t=SPI_ReadByte();//接收响应
		if((t&0x1F)!=0x05)return 2;//响应错误
	}
	return 0;//写入成功
}



//向SD卡发送一个命令
//输入: u8 cmd   命令
//      u32 arg  命令参数
//      u8 crc   crc校验值
//返回值:SD卡返回的响应
u8 SD_SendCmd(u8 cmd, u32 arg, u8 crc)
{
	u8 r1;
	u8 Retry=0;
	
	SD_DisSelect();//取消上次片选
	if(SD_Select())return 0XFF;//片选失效
	//发送
	SPI_WriteByte(cmd | 0x40);//分别写入命令
	SPI_WriteByte(arg >> 24);
	SPI_WriteByte(arg >> 16);
	SPI_WriteByte(arg >> 8);
	SPI_WriteByte(arg);
	SPI_WriteByte(crc);
	if(cmd==CMD12)SPI_WriteByte(0xff);//Skip a stuff byte when stop reading
	//等待响应，或超时退出

	Retry=0X1F;
	do
	{
		r1=SPI_ReadByte();
	}
	while((r1&0X80) && Retry--);
	//返回状态值
	return r1;
}



//获取SD卡的CID信息，包括制造商信息
//输入: u8 *cid_data(存放CID的内存，至少16Byte）
//返回值:0：NO_ERR
//		 1：错误
u8 SD_GetCID(u8 *cid_data)
{
	u8 r1;
	//发CMD10命令，读CID
	r1=SD_SendCmd(CMD10,0,0x01);
	if(r1==0x00)
	{
		r1=SD_RecvData(cid_data,16);//接收16个字节的数据
	}
	SD_DisSelect();//取消片选
	if(r1)return 1;
	else return 0;
}



//获取SD卡的CSD信息，包括容量和速度信息
//输入:u8 *cid_data(存放CID的内存，至少16Byte）
//返回值:0：NO_ERR
//		 1：错误
u8 SD_GetCSD(u8 *csd_data)
{
	u8 r1;
	r1=SD_SendCmd(CMD9,0,0x01);//发CMD9命令，读CSD
	if(r1==0)
	{
		r1=SD_RecvData(csd_data, 16);//接收16个字节的数据
	}
	SD_DisSelect();//取消片选
	if(r1)return 1;
	else return 0;
}



//获取SD卡的总扇区数（扇区数）
//返回值:0： 取容量出错
//其他:SD卡的容量(扇区数/512字节)
//每扇区的字节数必为512，因为如果不是512，则初始化不能通过.
u32 SD_GetSectorCount(void)
{
	u8 csd[16];
	u32 Capacity;
	u8 n;
	u16 csize;
	//取CSD信息，如果期间出错，返回0
	if(SD_GetCSD(csd)!=0) return 0;
	//如果为SDHC卡，按照下面方式计算
	if((csd[0]&0xC0)==0x40)	 //V2.00的卡
	{
		csize = csd[9] + ((u16)csd[8] << 8) + 1;
		Capacity = (u32)csize << 10;//得到扇区数
	}
	else //V1.XX的卡
	{
		n = (csd[5] & 15) + ((csd[10] & 128) >> 7) + ((csd[9] & 3) << 1) + 2;
		csize = (csd[8] >> 6) + ((u16)csd[7] << 2) + ((u16)(csd[6] & 3) << 10) + 1;
		Capacity= (u32)csize << (n - 9);//得到扇区数
	}
	return Capacity;
}



//初始化SD卡
u8 SD_Initialize(void)
{
	u8 r1;      // 存放SD卡的返回值
	u16 retry;  // 用来进行超时计数
	u8 buf[4];
	u16 i;
	u16 try;

	SPI3_Configuration();
	SD_SPI_SpeedLow();	//设置到低速模式
	Delay(5000);//SPI总线初始化延时一会再初始化SD卡

	for(i=0; i<15; i++)SPI_WriteByte(0XFF); //发送最少74个脉冲
	
	retry=200;
	do
	{
		r1=SD_SendCmd(CMD0,0,0x95);//进入IDLE状态
	}
	while((r1!=0X01) && retry--);
	
	SD_Type=0;//默认无卡
	if(r1==0X01)
	{
		if(SD_SendCmd(CMD8,0x1AA,0x87)==1)//SD V2.0
		{
			for(i=0; i<4; i++)buf[i] = SPI_ReadWriteByte(0XFF);	//Get trailing return value of R7 resp
			if(buf[2]==0X01&&buf[3]==0XAA)//卡是否支持2.7~3.6V
			{
				retry=0XFFFE;
				do
				{
					SD_SendCmd(CMD55,0,0X01);	//发送CMD55
					r1=SD_SendCmd(CMD41,0x40000000,0X01);//发送CMD41
				}
				while(r1&&retry--);
				if(retry&&SD_SendCmd(CMD58,0,0X01)==0)//鉴别SD2.0卡版本开始
				{
					for(i=0; i<4; i++)buf[i] = SPI_ReadWriteByte(0XFF); //得到OCR值
					if(buf[0]&0x40)SD_Type=SD_TYPE_V2HC;    //检查CCS
					else SD_Type=SD_TYPE_V2;
				}
			}
		}
		else //SD V1.x/ MMC	V3
		{
			SD_SendCmd(CMD55,0,0X01);		//发送CMD55
			r1=SD_SendCmd(CMD41,0,0X01);	//发送CMD41
			if(r1<=1)
			{
				SD_Type=SD_TYPE_V1;
				retry=0XFFFE;
				do //等待退出IDLE模式
				{
					SD_SendCmd(CMD55,0,0X01);	//发送CMD55
					r1=SD_SendCmd(CMD41,0,0X01);//发送CMD41
				}
				while(r1&&retry--);
			}
			else
			{
				SD_Type=SD_TYPE_MMC;//MMC V3
				retry=0XFFFE;
				do //等待退出IDLE模式
				{
					r1=SD_SendCmd(CMD1,0,0X01);//发送CMD1
				}
				while(r1&&retry--);
			}
			if(retry==0||SD_SendCmd(CMD16,512,0X01)!=0)SD_Type=SD_TYPE_ERR;//错误的卡
		}
	}
	SD_DisSelect();//取消片选
	SD_SPI_SpeedHigh();//SPI配置为高速模式
	
	if (SD_Type)
		return 0;
	else if(r1)
		return r1;
	return (0xaa);//其他错误
}



//读SD卡
//buf:数据缓存区
//sector:扇区
//cnt:扇区数
//返回值:0,ok;其他,失败.
u8 SD_ReadDisk(u8*buf,u32 sector,u8 cnt)
{
	u8 r1;
	if(SD_Type!=SD_TYPE_V2HC)sector <<= 9;//转换为字节地址
	if(cnt==1)
	{
		r1=SD_SendCmd(CMD17,sector,0X01);//读命令
		if(r1==0)//指令发送成功
		{
			r1=SD_RecvData(buf,512);//接收512个字节
		}
	}
	else
	{
		r1=SD_SendCmd(CMD18,sector,0X01);//连续读命令
		do
		{
			r1=SD_RecvData(buf,512);//接收512个字节
			buf+=512;
		}
		while(--cnt && r1==0);
		SD_SendCmd(CMD12,0,0X01);	//发送停止命令
	}
	SD_DisSelect();//取消片选
	return r1;//
}



//写SD卡
//buf:数据缓存区
//sector:起始扇区
//cnt:扇区数
//返回值:0,ok;其他,失败.
u8 SD_WriteDisk(u8*buf,u32 sector,u8 cnt)
{
	u8 r1;
	if(SD_Type!=SD_TYPE_V2HC)sector *= 512;//转换为字节地址
	if(cnt==1)
	{
		r1=SD_SendCmd(CMD24,sector,0X01);//读命令
		if(r1==0)//指令发送成功
		{
			r1=SD_SendBlock(buf,0xFE);//写512个字节
		}
	}
	else
	{
		if(SD_Type!=SD_TYPE_MMC)
		{
			SD_SendCmd(CMD55,0,0X01);
			SD_SendCmd(CMD23,cnt,0X01);//发送指令
		}
		r1=SD_SendCmd(CMD25,sector,0X01);//连续读命令
		if(r1==0)
		{
			do
			{
				r1=SD_SendBlock(buf,0xFC);//接收512个字节
				buf+=512;
			}
			while(--cnt && r1==0);
			r1=SD_SendBlock(0,0xFD);//接收512个字节
		}
	}
	SD_DisSelect();//取消片选
	
	return r1;//
}

//SD测试任务
void SDCardReceiveTask(void* param)
{
	if(SD_Initialize()==0)
	{
	
		f_mount(0, &fs); 					
		res = f_open(&fil,"0:/test.txt", FA_OPEN_ALWAYS|FA_WRITE);					       										
	}
	while(1)
	{
		res = f_lseek (&fil, f_size(&fil));							
		res = f_write(&fil, startline, strlen(startline), &bww);		
		GPIO_ResetBits(GPIOA,GPIO_Pin_14); 
		delay_ms(20);  		   
		GPIO_SetBits(GPIOA,GPIO_Pin_14);	
		delay_ms(20); 
	}
}

//SD测试任务 测试成功
void SDCardTestTask(void* param)
{
	//测试f_lseek()能否扩展大文件/f_write()
	int i = 0;
	while(SD_Initialize()!=0)
	{			    
		
	}
	
	GPIO_ResetBits(GPIOB,GPIO_Pin_9); 
	f_mount(0, &fs); 					
	res = f_open(&fil,"0:/1test.txt", FA_OPEN_ALWAYS|FA_WRITE);	
	
	while(res == FR_OK)
	{
		//printf("i%d\n", i);
		
		res = f_lseek (&fil, f_size(&fil));							
		res = f_write(&fil, startline, strlen(startline), &bww);		
		
		GPIO_ResetBits(GPIOA,GPIO_Pin_14); 
		delay_ms(2);  		   
		GPIO_SetBits(GPIOA,GPIO_Pin_14);	
		delay_ms(2); 

		res = f_sync(&fil);
		
		i++;
	}
}



//启动文件系统与SD卡
void ReadyToWrite()
{
	  //检查SD卡初始化情况准备写入数据
		if(SD_Initialize()==0)
		{
			//搭载文件系统
			res = f_mount(0,&fs); 
			
			//打卡文件
			res = f_open(&fil, FILENAME , FA_OPEN_ALWAYS|FA_WRITE|FA_WRITE);	
			
			//写入测试行
			res = f_lseek (&fil,f_size(&fil));						
			res = f_write(&fil, startline, strlen(startline), &bww);
			
			res = f_sync(&fil);		
			
			//判断是否成功写入
			if(res == FR_OK)
			{
				GPIO_ResetBits(GPIOA,GPIO_Pin_14); 
				GPIO_ResetBits(GPIOB,GPIO_Pin_9);
				delay_ms(500);  		   
				GPIO_SetBits(GPIOA,GPIO_Pin_14);	   
				GPIO_SetBits(GPIOB,GPIO_Pin_9);
//				printf("write success!");
			}
											
		}
}


// 打开文件res = f_open(&fil, FILENAME, FA_OPEN_EXISTING|FA_WRITE|FA_READ); 放在外面！！！
//写入一行数据 加快写的速度，并且文件处于一直开启状态
void WriteDataToSDOneLine(const state_t *state, setpoint_t *setpoint, control_t *control, const u32 tick)
{
	//res = f_open(&fil, FILENAME, FA_OPEN_ALWAYS|FA_WRITE|FA_READ);
	
	//转换数据格式：时钟 当前横滚角 遥控横滚角 控制量
	sprintf(data_row, "%u\t%10.6f\t%10.6f\t%10.6f \n", tick, state->attitude.roll, setpoint->custom, control->delta_control);
	
	//写入数据
	res = f_lseek (&fil,f_size(&fil));								
	res = f_write(&fil, data_row, strlen(data_row), &bww);	

	//是否关闭成功
	res = f_sync(&fil);
}

//写入数据
//TODO:可以写入自己需要的数据
void WriteDataToSD(const state_t *state, setpoint_t *setpoint, control_t *control, const u32 tick)
{
	//转换数据格式
	sprintf(timestamp, "%u\t", tick);
	sprintf(state_attitude, "%10.6f\t", state->attitude.roll);
	sprintf(set_attitude, "%10.6f\t", setpoint->custom);
	sprintf(control_value, "%10.6f\n", control->delta_control);
	
	//打开文件
	res = f_open(&fil, FILENAME, FA_OPEN_ALWAYS|FA_WRITE|FA_READ);
	
	//写入数据
	res = f_lseek (&fil,f_size(&fil));								
	res = f_write(&fil, timestamp, strlen(timestamp), &bww);	
	res = f_lseek (&fil,f_size(&fil));								
	res = f_write(&fil, state_attitude, strlen(state_attitude), &bww);
	res = f_lseek (&fil,f_size(&fil));								
	res = f_write(&fil, set_attitude, strlen(set_attitude), &bww);
	res = f_lseek (&fil,f_size(&fil));								
	res = f_write(&fil, control_value, strlen(control_value), &bww);
	//是否写入成功
	if(res == FR_OK)
	{
//		printf("tick %u",tick);
	}
	//是否关闭成功
	res = f_close(&fil);
	if(res == FR_OK)
	{
//		printf("close");
	}
						
}


//当前的姿态角
void WriteStateAttitude(const state_t *state, const u32 tick)
{
}
//控制的姿态角
void WriteSetpointAttitude(setpoint_t *setpoint, const u32 tick)
{
}
//控制量
void WriteControlValue(control_t *control, const u32 tick)
{
}
//时间戳
void WriteTimestamp()
{
}
