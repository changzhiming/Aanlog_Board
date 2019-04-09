#include "platform.h"
#include "w25qxx.h"
#include "stm32f4xx_hal.h"
#include "flash_if.h"
#include "string.h"
#include "lcd1602.h"
/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbutils.h"
#include "mbcrc.h"
#include "lwip.h"
/* ----------------------- system function ----------------------------------*/
static u8 DA_OUT(u8 ch,u8 rng,u8 data);
static void InitTLC5620(void);
static void Save_Out_State(void);
static void Get_Out_Addr(void);
/* ----------------------- system var ----------------------------------*/
static char device_name[30] = "Analog_board_V2_5";
/*
5 检测系统RAM
*/
SYSINFO_TypeDef SysConfigure;        //系统配置信息

static TimeTable_TypeDef TimeTable[500];    //时间表存储
static uint8_t clock_sync = 0;   //时钟同步
static uint32_t lastTick = 0;         //时间检测
static uint32_t buf_size = 0;         //数据包位置
static uint8_t  aucRtuBuf[256];       //透传Modbus rtu buf
static uint8_t  temp_value = 0;

static uint32_t out_save_addr = FLASH_ADDR_OUT_STATA;       //输出保存地址
static uint32_t data_addr = FLASH_DATA_ADDR;
static uint8_t  wwdg_time = 0;      //定时喂狗时间
static uint16_t ADC_Value[800] = {0};         //adc
static char sys_buf[2048];                   //系统配置buf
static uint16_t sys_buf_size = 0;

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart1;
extern DAC_HandleTypeDef hdac;
extern RTC_HandleTypeDef hrtc;

#define huart485 huart3
#define huart_debug huart1
#define USART485 USART3
/* ----------------------- Holding register Defines ------------------------------------------*/

#define REG_HOLDING_START 1U
#define REG_HOLDING_NREGS 48

static unsigned short usRegHoldingStart = REG_HOLDING_START;
static unsigned short usRegHoldingBuf[REG_HOLDING_NREGS]={0};

/* ----------------------- input register Defines ------------------------------------------*/
#define REG_INPUT_START 1U
#define REG_INPUT_NREGS 1

static unsigned short usRegInputStart = REG_INPUT_START;
static unsigned short usRegInputBuf[REG_INPUT_NREGS]={0};


/* ----------------------- coils register Defines ------------------------------------------*/
#define REG_COILS_START     1U
#define REG_COILS_SIZE      1

static unsigned char ucRegCoilsBuf[REG_COILS_SIZE % 8 == 0 ? REG_COILS_SIZE / 8 : REG_COILS_SIZE / 8 +1]={0x00};	  //数量低于8个还需要验证一下，是否需要加1呢。

/* ----------------------- discrete register Defines ------------------------------------------*/
#define REG_DISC_START     1U
#define REG_DISC_SIZE      1

static unsigned char ucRegDiscBuf[REG_DISC_SIZE % 8 == 0 ? REG_DISC_SIZE / 8 : REG_DISC_SIZE / 8 +1] = { 0x00};	   //数量8的整数倍。

//更新继电器状态
static void updateRegCoilsCB()
{

}

//更新开关量输入状态
static void updateRegDiscreteCB()
{
}
//更新输出状态 0.00精度  24个
static void updateRegHoldingCB()
{
	int index = 24;
	HAL_GPIO_WritePin(D_OUT1_GPIO_Port,  D_OUT1_Pin, (GPIO_PinState)(usRegHoldingBuf[index++] & 0x01));
	HAL_GPIO_WritePin(D_OUT2_GPIO_Port,  D_OUT2_Pin, (GPIO_PinState)(usRegHoldingBuf[index++] & 0x01));
	HAL_GPIO_WritePin(D_OUT3_GPIO_Port,  D_OUT3_Pin, (GPIO_PinState)(usRegHoldingBuf[index++] & 0x01));
	HAL_GPIO_WritePin(D_OUT4_GPIO_Port,  D_OUT4_Pin, (GPIO_PinState)(usRegHoldingBuf[index++] & 0x01));
	
	for(; index < 32; index++)
	{
		DA_OUT(index - 28 , 2, usRegHoldingBuf[index] / 100.0 * 25);
	}
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, usRegHoldingBuf[index++] / 100.0 * 3.3 / 20  * (4095 / 3.3));
}
//更新输入状态 0.00精度  24个
static void updateRegInputCB()
{
	uint8_t * chandle = (uint8_t *)&SysConfigure.In_Chandle_9;
	
	int index = 0;
	usRegHoldingBuf[index++]= HAL_GPIO_ReadPin(D_IN8_GPIO_Port, D_IN8_Pin) ? 0 : 1;
	usRegHoldingBuf[index++]= HAL_GPIO_ReadPin(D_IN7_GPIO_Port, D_IN7_Pin) ? 0 : 1;
	usRegHoldingBuf[index++]= HAL_GPIO_ReadPin(D_IN6_GPIO_Port, D_IN6_Pin) ? 0 : 1;
	usRegHoldingBuf[index++]= HAL_GPIO_ReadPin(D_IN5_GPIO_Port, D_IN5_Pin) ? 0 : 1;
	usRegHoldingBuf[index++]= HAL_GPIO_ReadPin(D_IN4_GPIO_Port, D_IN4_Pin) ? 0 : 1;
	usRegHoldingBuf[index++]= HAL_GPIO_ReadPin(D_IN3_GPIO_Port, D_IN3_Pin) ? 0 : 1;
	usRegHoldingBuf[index++]= HAL_GPIO_ReadPin(D_IN2_GPIO_Port, D_IN2_Pin) ? 0 : 1;
	usRegHoldingBuf[index++]= HAL_GPIO_ReadPin(D_IN1_GPIO_Port, D_IN1_Pin) ? 0 : 1;
	
	volatile int adc_value = 0;
	int i, size;
	for(i = index; i < 16; i++)
	{
		adc_value = 0;
		for(size = 0; size < 100; size++)
		{
			adc_value = adc_value + ADC_Value[size * 8 + i - 8];
		}
		usRegHoldingBuf[i] = (adc_value * 3.3 / 4096)* 3.0769;
	}
	for(i = index; i < 16; i++)
	{
		switch(*(chandle + i - 8))
		{
			case 0:
				usRegHoldingBuf[i] = usRegHoldingBuf[i] > 500 ? 0 : 1;
				break;
			case 1:
				usRegHoldingBuf[i] = usRegHoldingBuf[i];
				break;
			case 2:
				usRegHoldingBuf[i] = usRegHoldingBuf[i] * 2;
				break;
			case 3:
				usRegHoldingBuf[i] = usRegHoldingBuf[i] > 900 ? 0 : 1000 * 20 * usRegHoldingBuf[i] / 100 / (29.2 - 3 * usRegHoldingBuf[i] / 100);    //   100 * (100 * usRegHoldingBuf[i] / 100 - 144.5) / (14.45 - 20 * usRegHoldingBuf[i] / 100); //;usRegHoldingBuf[i] > 500 ? 0 : 10.0 * usRegHoldingBuf[i] / (14.45 - usRegHoldingBuf[i] / 100.0);
				break;
		}
	}
}
eMBErrorCode eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;
	
    if( ( usAddress >= REG_HOLDING_START ) &&
        ( usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegHoldingStart );
        switch ( eMode )
        {
                /* Pass current register values to the protocol stack. */
            case MB_REG_READ:
							  updateRegInputCB();
                while( usNRegs > 0 )
                {
                    *pucRegBuffer++ =
                        ( unsigned char )( usRegHoldingBuf[iRegIndex] >> 8 );
                    *pucRegBuffer++ =
                        ( unsigned char )( usRegHoldingBuf[iRegIndex] &
                                           0xFF );
                    iRegIndex++;
                    usNRegs--;
                }
                break;

                /* Update current register values with new values from the
                 * protocol stack. */
            case MB_REG_WRITE:
                while( usNRegs > 0 )
                {
                    usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                    usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                    iRegIndex++;
                    usNRegs--;
                }
								updateRegHoldingCB();
								Save_Out_State();
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

eMBErrorCode eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_INPUT_START )
        && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
				updateRegInputCB();       //更新输入寄存器状态
        iRegIndex = ( int )( usAddress - usRegInputStart );
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ =
                ( unsigned char )( usRegInputBuf[iRegIndex] & 0xFF );
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

eMBErrorCode eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iNCoils = ( int )usNCoils;
    unsigned short  usBitOffset;

    /* Check if we have registers mapped at this block. */
    if( ( usAddress >= REG_COILS_START ) &&
        ( usAddress + usNCoils <= REG_COILS_START + REG_COILS_SIZE ) )
    {
        usBitOffset = ( unsigned short )( usAddress - REG_COILS_START );
        switch ( eMode )
        {
                /* Read current values and pass to protocol stack. */
            case MB_REG_READ:
                while( iNCoils > 0 )
                {
                    *pucRegBuffer++ = xMBUtilGetBits( ucRegCoilsBuf, usBitOffset,( unsigned char )( iNCoils > 8 ? 8 : iNCoils ) );
                    iNCoils -= 8;
                    usBitOffset += 8;
                }
                break;

                /* Update current register values. */
            case MB_REG_WRITE:
                while( iNCoils > 0 )
                {
                    xMBUtilSetBits( ucRegCoilsBuf, usBitOffset, ( unsigned char )( iNCoils > 8 ? 8 : iNCoils ), *pucRegBuffer++ );
                    iNCoils -= 8;
                    usBitOffset += 8;
                }
								updateRegCoilsCB();
								Save_Out_State();
                break;
        }

    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

eMBErrorCode eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    short           iNDiscrete = ( short )usNDiscrete;
    unsigned short  usBitOffset;

    /* Check if we have registers mapped at this block. */
    if( ( usAddress >= REG_DISC_START ) &&
        ( usAddress + usNDiscrete <= REG_DISC_START + REG_DISC_SIZE ) )
    {
				updateRegDiscreteCB(); //更新开关量输入值;
        usBitOffset = ( unsigned short )( usAddress - REG_DISC_START );
        while( iNDiscrete > 0 )
        {
            *pucRegBuffer++ =
                xMBUtilGetBits( ucRegDiscBuf, usBitOffset,
                                ( unsigned char )( iNDiscrete >
                                                   8 ? 8 : iNDiscrete ) );
            iNDiscrete -= 8;
            usBitOffset += 8;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

//保存输出状态
static void Save_Out_State(void)
{ 
	if(out_save_addr % 4096 < sizeof(usRegHoldingBuf))
	{
		W25QXX_Erase_Sector(out_save_addr / 4096);         //擦除这个扇区
	}
	if(data_addr % 4096 < sizeof(data_addr))
	{
		W25QXX_Erase_Sector(data_addr / 4096);         //擦除这个扇区
	}
	
	W25QXX_Write((uint8_t *)usRegHoldingBuf, out_save_addr, sizeof(usRegHoldingBuf));
	W25QXX_Write((uint8_t *)&out_save_addr, data_addr, sizeof(out_save_addr));
	data_addr += sizeof(data_addr);
	out_save_addr += sizeof(usRegHoldingBuf);
	
	if(out_save_addr + sizeof(usRegHoldingBuf) >= FLASH_ADDR_END)
	{
		out_save_addr = FLASH_ADDR_OUT_STATA;
	}
	if(data_addr + sizeof(data_addr) >= FLASH_ADDR_OUT_STATA)
	{
		data_addr = FLASH_DATA_ADDR;
	}
}
//还原输出状态
static void Get_Out_Addr(void)
{
	volatile int i = 0;
	uint32_t value = 0;

	while(1)
	{
		if(FLASH_DATA_ADDR + i* 4 >= FLASH_ADDR_OUT_STATA)
		{
			data_addr = FLASH_DATA_ADDR;
			out_save_addr = FLASH_ADDR_OUT_STATA;
			break;
		}
		W25QXX_Read((uint8_t *)&value, FLASH_DATA_ADDR + (i++)* 4, sizeof(value));
		if(value == 0xFFFFFFFF)
		{
			if(i < 2)
			{
				data_addr = FLASH_DATA_ADDR;
				out_save_addr = FLASH_ADDR_OUT_STATA;
				break;
			}
			data_addr = FLASH_DATA_ADDR + (i - 2) * 4;
			
			W25QXX_Read((uint8_t *)&out_save_addr, data_addr, sizeof(out_save_addr));
			break;
		}
	}
	if(data_addr != FLASH_DATA_ADDR)
	{
		W25QXX_Read((uint8_t *)usRegHoldingBuf, out_save_addr, sizeof(usRegHoldingBuf));
		out_save_addr += sizeof(usRegHoldingBuf);
		
		updateRegHoldingCB();
	}
}

void RS485_Send(uint8_t * send_buf, uint16_t size)
{
	//先读取数据
	while(__HAL_UART_GET_FLAG(&huart485, UART_FLAG_RXNE) == SET) {
		uint8_t value;  HAL_UART_Receive(&huart485, &value, 1, 0xFF);
	}
	HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit_IT(&huart485, send_buf, size);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART485)
	{
		HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_RESET);
		HAL_UART_Receive_IT(&huart485, &temp_value, 1);       //重新启动接收数据
	}
}

void RS485_Recive(uint8_t ** recive_buf, uint8_t *size)
{
	if(((HAL_GetTick() - lastTick) >= 5) && buf_size >= 4)
	{
		*size = buf_size;
		*recive_buf = &aucRtuBuf[0];
		buf_size = 0;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART485)
	{
		if(((HAL_GetTick() - lastTick) > 5 && buf_size >= 4) || buf_size > 255)
		{
			buf_size = 0;
		}
		lastTick = HAL_GetTick();
		aucRtuBuf[buf_size++] = temp_value;
		
		HAL_UART_Receive_IT(&huart485, &temp_value, 1);
	}
}


int fputc(int ch, FILE *f)
{
//  RS485_Send((uint8_t *)&ch, 1);
//	HAL_Delay(1);
  return ch;
}

int fgetc(FILE *f)
{
	uint8_t  ch;     
	HAL_UART_Receive(&huart_debug,(uint8_t *)&ch, 1, 0xFFFF);
	return  ch;
}

#define TLC5620_SCLK(SET) HAL_GPIO_WritePin(DA_CLK_GPIO_Port, DA_CLK_Pin, (GPIO_PinState)(SET))	  // TLC5620时钟管脚
#define TLC5620_SDAA(SET) HAL_GPIO_WritePin(DA_DATA_GPIO_Port, DA_DATA_Pin, (GPIO_PinState)(SET))		// TLC5620数据管教
#define TLC5620_LOAD(SET) HAL_GPIO_WritePin(DA_LOAD_GPIO_Port, DA_LOAD_Pin, (GPIO_PinState)(SET))	  // TLC5620 LOAD管教
#define TLC5620_LDAC(SET) HAL_GPIO_WritePin(DA_LD_GPIO_Port, DA_LD_Pin, (GPIO_PinState)(SET))	// TLC5620 LDAC管教

#define TLC5620_OUTA 0x0000
#define TLC5620_OUTB 0x4000
#define TLC5620_OUTC 0x8000
#define TLC5620_OUTD 0xc000

#define TLC5620_OUT_one    0x0000
#define TLC5620_OUT_double 0x2000

static void InitTLC5620(void)
{
	TLC5620_SCLK(0);
	TLC5620_SDAA(0);
	TLC5620_LOAD(1);
	TLC5620_LOAD(1);
}
//TLC 5620电压输出
//
static void  TLC5620_Conversion (u16 tlc5620_data)
{
    u8   m=0;
    u16  n;
    for(; m<0x0b; m++)
    {
        TLC5620_SCLK(1);
        n=tlc5620_data;
        n=n&0x8000;
        if(n==0x8000)
            TLC5620_SDAA(1);
        else
            TLC5620_SDAA(0);
        TLC5620_SCLK(0);
        tlc5620_data<<=1;
    }
    TLC5620_LOAD(0);
    TLC5620_LOAD(1);
    TLC5620_LDAC(0);
    TLC5620_LDAC(1);
}
//ch = 0 TLC5620_OUTA;1 TLC5620_OUTB;2 TLC5620_OUTC;3 TLC5620_OUTD;other err;
//rng= 0 TLC5620_OUT_one;other TLC5620_OUT_double;
//VO(DACA|B|C|D)+REF*(CODE/256)*(1+RNG)
static u8 DA_OUT(u8 ch,u8 rng,u8 data)
{
    u16 out_data=0;
    
    out_data |= data;
    out_data <<= 5;
    if(ch == 0){
        out_data |= TLC5620_OUTA;
    }else if(ch == 1){
        out_data |= TLC5620_OUTB;
    }else if(ch == 2){
        out_data |= TLC5620_OUTC;
    }else if(ch == 3){
        out_data |= TLC5620_OUTD;
    }else{
        return 0;
    }
    
    if(rng == 1){
        out_data |= TLC5620_OUT_one;
    }else if(rng == 2){
        out_data |= TLC5620_OUT_double;
    }else{
        out_data |= TLC5620_OUT_one;
    }
    
    TLC5620_Conversion(out_data);
    return 1;
}

void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef* hwwdg)
{
	if(wwdg_time <= 200)
	{
		HAL_WWDG_Refresh(hwwdg);
		wwdg_time++;
	}
}

/*void ethernetif_notify_conn_changed(struct netif *netif)
{
  if(netif_is_link_up(netif))
  {
    netif_set_up(netif);     
  }
  else
  {
    netif_set_down(netif);
  }
}*/

void WWDG_Refresh()
{
	wwdg_time = 0;
	struct pbuf *buf = pbuf_alloc(PBUF_TRANSPORT, 1024, PBUF_RAM);
	if(buf == NULL)
		Soft_Reset();
	pbuf_free(buf);
}

void Soft_Reset(void)
{
	__set_FAULTMASK(1);
	NVIC_SystemReset();
}

//tftp
uint8_t file_handle = 0;
//1->IAP, 2->Setting, 3->ReadSetting, 4->Reset, 5->Clear, 6->SetTime
uint32_t flash_addr_firmware = FLASH_ADDR_FIRMWARE;

void* tftp_open(const char* fname, const char* mode, u8_t write)
{
	file_handle = 0;
	if(strstr(fname, "IAP") != NULL) 
	{
		flash_addr_firmware = FLASH_ADDR_FIRMWARE;
		file_handle = 1;
	}
	
	if(strstr(fname, "Setting") != NULL)
	{
		file_handle = 2;
		memset(sys_buf, 0, sizeof(sys_buf));
		sys_buf_size = 0;
	}
	
	if(strstr(fname, "ReadSetting") != NULL)
	{
		file_handle = 3;
		memset(sys_buf, 0, sizeof(sys_buf));
		sys_buf_size = sprintf(sys_buf, "[General]\r\nip=%d.%d.%d.%d\r\nnetmask=%d.%d.%d.%d\r\nmac=%02X.%02X.%02X.%02X.%02X.%02X\r\nband=%d\r\nstop=%d\r\nwordlength=%d\r\nparity=%d\r\nmodbusaddr=%d\r\nversion=%d\r\n",
		SysConfigure.IP[0], SysConfigure.IP[1], SysConfigure.IP[2], SysConfigure.IP[3],
		SysConfigure.Net_Mask[0], SysConfigure.Net_Mask[1], SysConfigure.Net_Mask[2], SysConfigure.Net_Mask[3],
		SysConfigure.MACAddr[0], SysConfigure.MACAddr[1],SysConfigure.MACAddr[2], SysConfigure.MACAddr[3], SysConfigure.MACAddr[4], SysConfigure.MACAddr[5],
		SysConfigure.RS485_BaudRate, SysConfigure.RS485_STOPBITS, SysConfigure.RS485_WordLength, SysConfigure.RS485_Parity, SysConfigure.Modbus_Addr, SysConfigure.APP_Version);
		//
		sys_buf_size += sprintf(&sys_buf[sys_buf_size], "device_type=%s\r\n", device_name);
		sys_buf_size += sprintf(&sys_buf[sys_buf_size], "in_chandle_9=%d\r\nin_chandle_10=%d\r\nin_chandle_11=%d\r\nin_chandle_12=%d\r\nin_chandle_13=%d\r\nin_chandle_14=%d\r\nin_chandle_15=%d\r\nin_chandle_16=%d\r\n", 
		SysConfigure.In_Chandle_9, SysConfigure.In_Chandle_10, SysConfigure.In_Chandle_11, SysConfigure.In_Chandle_12, SysConfigure.In_Chandle_13, SysConfigure.In_Chandle_14, SysConfigure.In_Chandle_15,
		SysConfigure.In_Chandle_16);
	}
	if(strstr(fname, "Reset") != NULL)
	{
		file_handle = 4;
	}
	
	if(strstr(fname, "Clear") != NULL)
	{
		file_handle = 5;
	}
	
	if(strstr(fname, "SetTime") != NULL)
	{
		file_handle = 6;
		memset(sys_buf, 0, sizeof(sys_buf));
		sys_buf_size = 0;
	}
	return (void *)(&file_handle);
}

void tftp_close(void* handle)
{
	switch(*(uint8_t *)(handle))
	{
		case 1:
			if(usMBCRC16( ( UCHAR * )FLASH_ADDR_FIRMWARE, flash_addr_firmware - FLASH_ADDR_FIRMWARE) == 0)
			{
				SysConfigure.APP_Update = 1;
				SysConfigure.APP_Size = flash_addr_firmware - FLASH_ADDR_FIRMWARE;
				while(FLASH_If_Write(FLASH_ADDR_CONFIG, (uint8_t *)&SysConfigure, sizeof(SysConfigure)) != 0)
				{}
			}
			flash_addr_firmware = FLASH_ADDR_FIRMWARE;
			break;
		case 2:
			sys_buf_size = 0;
			Sys_Analysis();
			break;
		case 3:
			sys_buf_size = 0;
			break;
		case 4:
			Soft_Reset();
			break;
		case 5:
			ClearSysInfo();
			break;
		case 6:
			TimeTable_Analysis();
			sys_buf_size = 0;
			break;
	}
	*(uint8_t *)(handle) = 0;
}
int tftp_read(void* handle, void* buf, int bytes)
{
	switch(*(uint8_t *)(handle))
	{
		case 3:
			memcpy(buf, sys_buf, sys_buf_size);
			break;
	}
	return sys_buf_size;
}

static uint8_t write_buf[1024];
int tftp_write(void* handle, struct pbuf* p)
{
	int size = 0, len = 0;
	if(p != NULL)
	{
		for(struct pbuf* q = p; q != NULL; q = q->next)
		{
			//判断要操作的类型
			switch(*(uint8_t *)(handle))
			{
				case 1:	
					memset(write_buf, 0, sizeof(write_buf));
					len = q->len;
					memcpy(write_buf, q->payload, len);
					while(FLASH_If_Write(flash_addr_firmware, write_buf, len) != 0){}
					flash_addr_firmware += len;
					break;
				case 2:
					if(sizeof(sys_buf) > sys_buf_size + q->len)
					{
						memcpy(sys_buf, q->payload,  q->len);
						sys_buf_size += q->len;
						size += q->len;
					}
					break;
				case 6:
					if(sizeof(sys_buf) > sys_buf_size + q->len)
					{
						memcpy(sys_buf, q->payload, q->len);
						sys_buf_size += q->len;
						size += q->len;
					}
					break;
			}
		}
		pbuf_free(p);
	}
	return size;
}
void SysConfig(void )
{
	//端口初始化
	HAL_GPIO_WritePin(D_EN1_GPIO_Port,  D_EN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(A_EN1_GPIO_Port,  A_EN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(D_EN2_GPIO_Port,  D_EN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(A_EN2_GPIO_Port,  A_EN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(D_EN3_GPIO_Port,  D_EN3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(A_EN3_GPIO_Port,  A_EN3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(D_EN4_GPIO_Port,  D_EN4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(A_EN4_GPIO_Port,  A_EN4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(D_EN5_GPIO_Port,  D_EN5_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(A_EN5_GPIO_Port,  A_EN5_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(D_EN6_GPIO_Port,  D_EN6_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(A_EN6_GPIO_Port,  A_EN6_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(D_EN7_GPIO_Port,  D_EN7_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(A_EN7_GPIO_Port,  A_EN7_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(D_EN8_GPIO_Port,  D_EN8_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(A_EN8_GPIO_Port,  A_EN8_Pin, GPIO_PIN_RESET);
	
	if(SysConfigure.In_Chandle_9 == 0 || SysConfigure.In_Chandle_9 == 3) 
		HAL_GPIO_WritePin(D_EN1_GPIO_Port,  D_EN1_Pin, GPIO_PIN_SET);
	if(SysConfigure.In_Chandle_9 == 2) 
		HAL_GPIO_WritePin(A_EN1_GPIO_Port,  A_EN1_Pin, GPIO_PIN_SET);

	if(SysConfigure.In_Chandle_10 == 0 || SysConfigure.In_Chandle_10 == 3)
		HAL_GPIO_WritePin(D_EN2_GPIO_Port,  D_EN2_Pin, GPIO_PIN_SET);
	if(SysConfigure.In_Chandle_10 == 2)
		HAL_GPIO_WritePin(A_EN2_GPIO_Port,  A_EN2_Pin, GPIO_PIN_SET);
	
	if(SysConfigure.In_Chandle_11 == 0 || SysConfigure.In_Chandle_11 == 3)
		HAL_GPIO_WritePin(D_EN3_GPIO_Port,  D_EN3_Pin, GPIO_PIN_SET);
	if(SysConfigure.In_Chandle_11 == 2)
		HAL_GPIO_WritePin(A_EN3_GPIO_Port,  A_EN3_Pin, GPIO_PIN_SET);
	
	if(SysConfigure.In_Chandle_12 == 0 || SysConfigure.In_Chandle_12 == 3)
		HAL_GPIO_WritePin(D_EN4_GPIO_Port,  D_EN4_Pin, GPIO_PIN_SET);
	if(SysConfigure.In_Chandle_12 == 2)
		HAL_GPIO_WritePin(A_EN4_GPIO_Port,  A_EN4_Pin, GPIO_PIN_SET);
	
	if(SysConfigure.In_Chandle_13 == 0 || SysConfigure.In_Chandle_13 == 3)
		HAL_GPIO_WritePin(D_EN5_GPIO_Port,  D_EN5_Pin, GPIO_PIN_SET);
	if(SysConfigure.In_Chandle_13 == 2)
		HAL_GPIO_WritePin(A_EN5_GPIO_Port,  A_EN5_Pin, GPIO_PIN_SET);
	
	if(SysConfigure.In_Chandle_14 == 0 || SysConfigure.In_Chandle_14 == 3)
		HAL_GPIO_WritePin(D_EN6_GPIO_Port,  D_EN6_Pin, GPIO_PIN_SET);
	if(SysConfigure.In_Chandle_14 == 2)
		HAL_GPIO_WritePin(A_EN6_GPIO_Port,  A_EN6_Pin, GPIO_PIN_SET);
	
	if(SysConfigure.In_Chandle_15 == 0 || SysConfigure.In_Chandle_15 == 3)
		HAL_GPIO_WritePin(D_EN7_GPIO_Port,  D_EN7_Pin, GPIO_PIN_SET);
	if(SysConfigure.In_Chandle_15 == 2)
		HAL_GPIO_WritePin(A_EN7_GPIO_Port,  A_EN7_Pin, GPIO_PIN_SET);
	
	if(SysConfigure.In_Chandle_16 == 0 || SysConfigure.In_Chandle_16 == 3)
		HAL_GPIO_WritePin(D_EN8_GPIO_Port,  D_EN8_Pin, GPIO_PIN_SET);
	if(SysConfigure.In_Chandle_16 == 2)
		HAL_GPIO_WritePin(A_EN8_GPIO_Port,  A_EN8_Pin, GPIO_PIN_SET);
	
}
//init
void PLATFORM_Init(void )
{
	W25QXX_Init();
	
	memset((u8*)&SysConfigure, 0, sizeof(SysConfigure));
  memcpy((void *)&SysConfigure, (void *)FLASH_ADDR_CONFIG, sizeof(SysConfigure));//读取所有配置信息
	
	if(SysConfigure.Sysinfo_errror != 0x55) {
		SysConfigure.Modbus_Addr = 1;
		SysConfigure.IP[0] = 192;
		SysConfigure.IP[1] = 168;
		SysConfigure.IP[2] = 1;
		SysConfigure.IP[3] = 30;
		
		SysConfigure.Net_Mask[0] = 255;
		SysConfigure.Net_Mask[1] = 255;
		SysConfigure.Net_Mask[2] = 255;
		SysConfigure.Net_Mask[3] = 0;
		
		SysConfigure.MACAddr[0] = 0x00;
		SysConfigure.MACAddr[1] = 0x80;
		SysConfigure.MACAddr[2] = 0xE1;
		SysConfigure.MACAddr[3] = 0x00;
		SysConfigure.MACAddr[4] = 0x00;
		SysConfigure.MACAddr[5] = 0x11;
		
		SysConfigure.RS485_BaudRate = 9600;
		SysConfigure.RS485_STOPBITS = 1;
		SysConfigure.RS485_WordLength = 8;
		SysConfigure.RS485_Parity = 0;
		SysConfigure.APP_Update = 0;
		SysConfigure.APP_Size = 0;
		SysConfigure.APP_Version = 0;
		//输入通道
		SysConfigure.In_Chandle_9 = 0;
		SysConfigure.In_Chandle_10 = 0;
		SysConfigure.In_Chandle_11 = 0;
		SysConfigure.In_Chandle_12 = 0;
		SysConfigure.In_Chandle_13 = 0;
		SysConfigure.In_Chandle_14 = 0;
		SysConfigure.In_Chandle_15 = 0;
		SysConfigure.In_Chandle_16 = 0;
		
		SysConfigure.Sysinfo_errror = 0x55;
		
		while(FLASH_If_Write(FLASH_ADDR_CONFIG, (uint8_t *)&SysConfigure, sizeof(SysConfigure)) != 0 ){}
	}

	lastTick = HAL_GetTick();
	//485初始化
  huart485.Init.BaudRate = SysConfigure.RS485_BaudRate;
  huart485.Init.WordLength = SysConfigure.RS485_WordLength == 8 ? UART_WORDLENGTH_8B : UART_WORDLENGTH_9B;
  huart485.Init.StopBits = SysConfigure.RS485_STOPBITS == 1 ? UART_STOPBITS_1 : UART_STOPBITS_2;
	
	if(SysConfigure.RS485_Parity == 0)  {
		huart485.Init.Parity = UART_PARITY_NONE;
	} else if(SysConfigure.RS485_Parity == 1){
		huart485.Init.Parity = UART_PARITY_EVEN;
	} else {
		huart485.Init.Parity = UART_PARITY_ODD;
	}
  if (HAL_UART_Init(&huart485) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	
	HAL_UART_Receive_IT(&huart485, &temp_value, 1);        //开启485
	
	//tabel time
	memset((uint8_t *)TimeTable, 0, sizeof(TimeTable));
	W25QXX_Read((uint8_t *)TimeTable, FLASH_ADDR_SAVE_ADDR, sizeof(TimeTable));
	
	SysConfig();   //根据配置文件初始化采集类型
	//ADC初始化
	HAL_ADC_Start_DMA(&hadc3, (uint32_t*)&ADC_Value, 800);   //开启DMA
	//dac init
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	
	InitTLC5620();
	
	//io init
	Get_Out_Addr();
	//lcd init
	LCD1602_Init();
	UpdateLcdShow();
}
void UpdateLcdShow(void)
{
	static char str[20];
	sprintf(str, "%d.%d.%d.%d", SysConfigure.IP[0], SysConfigure.IP[1], SysConfigure.IP[2], SysConfigure.IP[3]);
	LCD1602_Show_Str(0, 0, (u8 *)str);
	//
	sprintf(str, "%d:%d:%d:%d:%d:%d", SysConfigure.RS485_BaudRate, SysConfigure.RS485_STOPBITS, SysConfigure.RS485_WordLength, SysConfigure.RS485_Parity, SysConfigure.Modbus_Addr, SysConfigure.APP_Version);
	LCD1602_Show_Str(0, 1, (u8 *)str);
}

void UpdateInOutShow(void)
{
	static char str[20];
	sprintf(str, "%d%d%d%d%d%d%d%d", (ucRegDiscBuf[0] >> 7) & 0x01, (ucRegDiscBuf[0] >> 6) & 0x01, (ucRegDiscBuf[0] >> 5) & 0x01, (ucRegDiscBuf[0] >> 4) & 0x01,
	(ucRegDiscBuf[0] >> 3) & 0x01, (ucRegDiscBuf[0] >> 2) & 0x01, (ucRegDiscBuf[0] >> 1) & 0x01,(ucRegDiscBuf[0] >> 0) & 0x01);
	LCD1602_Show_Str(0, 0, (u8 *)str);
	sprintf(str, "%d%d%d%d", (ucRegCoilsBuf[0] >> 2) & 0x01, (ucRegCoilsBuf[0] >> 1) & 0x01, (ucRegCoilsBuf[0] >> 0) & 0x01, (ucRegCoilsBuf[0] >> 3) & 0x01);
	LCD1602_Show_Str(0, 0, (u8 *)str);
}

void ClearSysInfo(void )
{
	static uint8_t temp[sizeof(SysConfigure)] = {0};
	while(FLASH_If_Write(FLASH_ADDR_CONFIG, (uint8_t *)temp, sizeof(SysConfigure)) != 0){}
}

static uint8_t time_send[20];
void Chandle_TimeTable(uint8_t hour, uint8_t min, uint8_t second)
{
	static USHORT usCRC16;
	int i = 0, size = 0;
	int num = sizeof(TimeTable) / sizeof(TimeTable[0]);
	
	if(clock_sync == 0)
		return;
	
	for(i = 0 ; i < num; i++)
	{
		if(TimeTable[i].mainId == 0)
			break;
		if(hour == TimeTable[i].hour && min == TimeTable[i].minute && second == TimeTable[i].second)
		{
			if(TimeTable[i].mainId == SysConfigure.Modbus_Addr)
			{
				if(TimeTable[i].type == 4 && TimeTable[i].addr >= 24 && TimeTable[i].addr < 48)
					usRegHoldingBuf[TimeTable[i].addr] =  TimeTable[i].value;
			}
			else
			{
				size = 0;
				time_send[size++] = TimeTable[i].mainId;
				if(TimeTable[i].type == 4)
				{
					time_send[size++] = 6;
					time_send[size++] = TimeTable[i].addr >> 8;
					time_send[size++] = TimeTable[i].addr & 0xFF;
					
					time_send[size++] = TimeTable[i].value >> 8;
					time_send[size++] = TimeTable[i].value & 0xFF;
					
				}
				if(TimeTable[i].type == 1)
				{
					time_send[size++] = 5;
					time_send[size++] = TimeTable[i].addr >> 8;
					time_send[size++] = TimeTable[i].addr & 0xFF;
					if(TimeTable[i].value == 1)
					{
						time_send[size++] = 0xFF;
						time_send[size++] = 00;
					}
					else
					{
						time_send[size++] = 00;
						time_send[size++] = 00;
					}
				}
				usCRC16 = usMBCRC16( ( UCHAR * ) (time_send), size);
				time_send[size++] = ( UCHAR )( usCRC16 & 0xFF );
				time_send[size++] = ( UCHAR )( usCRC16 >> 8 );
				
				RS485_Send(time_send, size);
				HAL_Delay(30);
				WWDG_Refresh();
			}
		}
	}
	updateRegHoldingCB();
}

void TimeTable_Analysis(void )
{
	static RTC_TimeTypeDef sTime;
  static RTC_DateTypeDef sDate;
	
	static int buf[10] = {0};
	char *pos = NULL;
	int i = 0, currentTimeNum = 0;
	int num = sizeof(TimeTable) / sizeof(TimeTable[0]);
	for(i = 0 ; i < num; i++)
	{
		if(TimeTable[i].mainId == 0) 
		{
			currentTimeNum = i;
			break;
		}
	}
	//time
	pos = strstr(sys_buf, "DataTime");
	if(pos)
	{
		sscanf(pos, "DataTime=%d:%d:%d:%d:%d:%d", &buf[0], &buf[1], &buf[2], &buf[3], &buf[4], &buf[5]);
		sTime.Hours = buf[3];  sTime.Minutes = buf[4];  sTime.Seconds = buf[5];
		sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
		sTime.StoreOperation = RTC_STOREOPERATION_RESET;
		HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		sDate.Month = buf[1];  sDate.Date = buf[3]; 	sDate.Year = buf[0];
		HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
		
		clock_sync = 1;
	}
	pos = strstr(sys_buf, "ClearTimeTable");
	if(pos)
	{
		memset((uint8_t *)TimeTable, 0, sizeof(TimeTable));
	}
	pos = strstr(sys_buf, "timeTable");
	while(pos)
	{
		pos = strstr(pos, "timeTable");
		if(pos)
		{
			pos = strstr(pos, "=");
			if(pos)
			{
				sscanf(pos,"=%d:%d:%d:%d:%d:%d:%d", &buf[0], &buf[1], &buf[2], &buf[3], &buf[4], &buf[5], &buf[6]);
				
				TimeTable[currentTimeNum].mainId = buf[0];
				TimeTable[currentTimeNum].type = buf[1];
				TimeTable[currentTimeNum].addr = buf[2];
				TimeTable[currentTimeNum].value = buf[3];
				TimeTable[currentTimeNum].hour = buf[4];
				TimeTable[currentTimeNum].minute = buf[5];
				TimeTable[currentTimeNum].second = buf[6];
				currentTimeNum++;
			}
		}
	}
	W25QXX_Write((uint8_t *)TimeTable, FLASH_ADDR_SAVE_ADDR, sizeof(TimeTable));
}

//解析系统配置信息
void Sys_Analysis(void)
{
	static int buf[10] = {0};
	char *pos = NULL;
	//ip
	pos = strstr(sys_buf, "ip");
	if(pos)
	{
		sscanf(pos,"ip=%d.%d.%d.%d", &buf[0], &buf[1], &buf[2], &buf[3]);
		SysConfigure.IP[0] = buf[0];
		SysConfigure.IP[1] = buf[1];
		SysConfigure.IP[2] = buf[2];
		SysConfigure.IP[3] = buf[3];
	}
	//net
	pos = strstr(sys_buf, "netmask");
	if(pos)
	{
		sscanf(pos,"netmask=%d.%d.%d.%d", &buf[0], &buf[1], &buf[2], &buf[3]);
		SysConfigure.Net_Mask[0] = buf[0];
		SysConfigure.Net_Mask[1] = buf[1];
		SysConfigure.Net_Mask[2] = buf[2];
		SysConfigure.Net_Mask[3] = buf[3];
	}
	
	//mac
	pos = strstr(sys_buf, "mac");
	if(pos)
	{
		sscanf(pos,"mac=%02X.%02X.%02X.%02X.%02X.%02X", &buf[0], &buf[1], &buf[2], &buf[3], &buf[4], &buf[5]);
		SysConfigure.MACAddr[0] = buf[0];
		SysConfigure.MACAddr[1] = buf[1];
		SysConfigure.MACAddr[2] = buf[2];
		SysConfigure.MACAddr[3] = buf[3];
		SysConfigure.MACAddr[4] = buf[4];
		SysConfigure.MACAddr[5] = buf[5];
	}
	
	//485 band
	pos = strstr(sys_buf, "band");
	if(pos)
	{
		sscanf(pos,"band=%d\r\n", &buf[0]);
		SysConfigure.RS485_BaudRate = buf[0];
	}
	//stop
	pos = strstr(sys_buf, "stop");
	if(pos)
	{
		sscanf(pos,"stop=%d\r\n", &buf[0]);
		SysConfigure.RS485_STOPBITS = buf[0];
	}
	
	//wordlength
	pos = strstr(sys_buf, "wordlength");
	if(pos)
	{
		sscanf(pos,"wordlength=%d\r\n", &buf[0]);
		SysConfigure.RS485_WordLength = buf[0];
	}
	
	//parity
	pos = strstr(sys_buf, "parity");
	if(pos)
	{
		sscanf(pos,"parity=%d\r\n", &buf[0]);
		SysConfigure.RS485_Parity = buf[0];
	}
	//modbusaddr
	pos = strstr(sys_buf, "modbusaddr");
	if(pos)
	{
		sscanf(pos,"modbusaddr=%d\r\n", &buf[0]);
		SysConfigure.Modbus_Addr = buf[0];
	}
	//version
	pos = strstr(sys_buf, "version");
	if(pos)
	{
		sscanf(pos,"version=%d", &buf[0]);
		SysConfigure.APP_Version = buf[0];
	}
	//IN CHandle 
	pos = strstr(sys_buf, "in_chandle_9");
	if(pos)
	{
		sscanf(pos,"in_chandle_9=%d", &buf[0]);
		SysConfigure.In_Chandle_9 = buf[0];
	}
	pos = strstr(sys_buf, "in_chandle_10");
	if(pos)
	{
		sscanf(pos,"in_chandle_10=%d", &buf[0]);
		SysConfigure.In_Chandle_10 = buf[0];
	}
	
	pos = strstr(sys_buf, "in_chandle_11");
	if(pos)
	{
		sscanf(pos,"in_chandle_11=%d", &buf[0]);
		SysConfigure.In_Chandle_11 = buf[0];
	}
	
	pos = strstr(sys_buf, "in_chandle_12");
	if(pos)
	{
		sscanf(pos,"in_chandle_12=%d", &buf[0]);
		SysConfigure.In_Chandle_12 = buf[0];
	}
	
	pos = strstr(sys_buf, "in_chandle_13");
	if(pos)
	{
		sscanf(pos,"in_chandle_13=%d", &buf[0]);
		SysConfigure.In_Chandle_13 = buf[0];
	}
	
	pos = strstr(sys_buf, "in_chandle_14");
	if(pos)
	{
		sscanf(pos,"in_chandle_14=%d", &buf[0]);
		SysConfigure.In_Chandle_14 = buf[0];
	}
	
	pos = strstr(sys_buf, "in_chandle_15");
	if(pos)
	{
		sscanf(pos,"in_chandle_15=%d", &buf[0]);
		SysConfigure.In_Chandle_15 = buf[0];
	}
	
	pos = strstr(sys_buf, "in_chandle_16");
	if(pos)
	{
		sscanf(pos,"in_chandle_16=%d", &buf[0]);
		SysConfigure.In_Chandle_16 = buf[0];
	}
	SysConfig();
	//写入数据
	while(FLASH_If_Write(FLASH_ADDR_CONFIG, (uint8_t *)&SysConfigure, sizeof(SysConfigure)) != 0){}
	//更新显示
	UpdateLcdShow();
}
