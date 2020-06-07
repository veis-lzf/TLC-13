#include "uarttl.h"
#include "delay.h"

// 全局变量
uint8_t gFingerSendBuf[20] = {0, 0, 0, 0, 0, 0, 0, 0}; // 指纹识别模块通讯的 待发送命令数据包，最大长度[20 byte]
uint8_t gFingerReceiveBuf[20] = {0, 0, 0, 0, 0, 0, 0, 0}; // 接收响应数据包，长度[20 byte]
uint8_t gZWbuf[3] = {0, 0, 0}; // 指纹特征ID缓冲区
uint8_t Finger_Img2Tz = 1; // 指纹特征模板（1， 2）
uint8_t RXRcnt = 0; // 计数变量，用于接收响应数据包计数

/**
 * 描述：完成：1.指纹图像的采集、2.提取特征模板1、3.比对指纹
 * 参数：无
 * 返回值：识别成功返回FTRUE(0xAA)，反之返回FALSE(0)
 */
uint8_t IsValid(void)
{
	uint8_t result;

	result = GetImg_F(); // 获取图像

	if(!result)
		return FALSE; // 获取图像失败

	Finger_Img2Tz = 1; // 提取特征存模板1
	Send_CMD_Handle(Img2Tz, 4);
	result = UARTReceive_Handle();

	if(result != FTRUE)
		return FALSE;

	Send_CMD_Handle(Search, 8); // 搜索指纹
	result = UARTReceive_Handle();

	if(result != FTRUE)
		return FALSE;

	return FTRUE;
}

// 清除指纹接收和发送缓冲区，包括计数变量清0
static void clr_Fingerbuf(void)
{
	uint8_t i;

	for (i = 0; i < 20; i++)
	{
		gFingerSendBuf[i] = 0;
		gFingerReceiveBuf[i] = 0;
	}

	RXRcnt = 0;
}

// 串口接收回调函数
// 作用：把模块返回的数据填入接收缓冲区
static uint8_t UART_Receive_Callback()
{
	int i, j;
	uint8_t len;

	for(i = 0; i < 50; i++)
	{
		delay_ms(10);

		if(RXRcnt >= 12)
			break;
	}

	if(i < 50)
	{
		for(j = 0; j < RXRcnt++; j++)
		{
			// 收到起始两个字节就等固定是
			if(gFingerReceiveBuf[j] == 0xEF
			    && gFingerReceiveBuf[j + 1] == 0x01)
			{
				break;
			}

			delay_ms(10);
		}

		len = gFingerReceiveBuf[j + 8];

		for(i = 0; i < 9 + len; i++)
		{
			gFingerReceiveBuf[i] = gFingerReceiveBuf[j];
			j++;
		}

		return TRUE;   //成功接收数据
	}

	else
	{
		return FALSE;
	}

}

/* 公有函数区域 */
// 串口初始化
void uart2_init(u32 bound)
{
	// GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	//使能USART2，GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	// USART2_TX   GPIOA2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// USART2_RX	  GPIOA3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Usart2 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3 ; // 抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		// 子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			// IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	// 根据指定的参数初始化VIC寄存器

	// USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

	USART_Init(USART2, &USART_InitStructure); //初始化串口2
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接受中断
	USART_Cmd(USART2, ENABLE);                    //使能串口2

}

// 串口2发送一个字节数据
void USART2_SendByte(const uint8_t ch)
{
	USART_SendData(USART2, ch);

	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
}

// 发送命令数据包
void SendDataByLen(uint8_t len)
{
	uint8_t i;

	for(i = 0; i < len; i++)
	{
		USART2_SendByte(gFingerSendBuf[i]);
	}
}

// 合成三种命令数据包
// 1.采集图像命令数据包(GetImg)12 byte
// 2.图像转特征命令数据包(Img2TZ)13 byte
// 3.搜索数据库比对指纹命令数据包(Search) 17 byte
void Send_CMD_Handle(uint8_t CMD, uint8_t len)
{
	uint16_t i;
	uint16_t sum = 0;
	clr_Fingerbuf();
	RXRcnt = 0;
	gFingerSendBuf[0] = 0xEF;
	gFingerSendBuf[1] = 0x01;
	gFingerSendBuf[2] = 0xFF;
	gFingerSendBuf[3] = 0xFF;
	gFingerSendBuf[4] = 0xFF;
	gFingerSendBuf[5] = 0xFF;
	gFingerSendBuf[6] = 0x01; // 指令
	gFingerSendBuf[7] = 0x00; // 7、8后面跟的数据长度
	gFingerSendBuf[8] = len;
	gFingerSendBuf[9] = CMD;

	switch(CMD)
	{
		// 在数据库搜索：
		// START ADDR PID LENGTH Command BufferID TempID TempCount SUM
		// 2 字节 4 字节 1 字节 2 字节 1 字节 1 字节 2 字节 2 字节 2 字节
		// 0xEF01 0xFFFFFFFF 0x01 0x0008 0x04 X XX XX Sum
		// 搜索指纹
		case Search:
		{
			gFingerSendBuf[10] = 0x01;
			gFingerSendBuf[11] = 0x00;
			gFingerSendBuf[12] = 0x00;
			gFingerSendBuf[13] = 0x01;
			gFingerSendBuf[14] = 0x2C;
			break;
		}

		//提取特征点ID0/ID1
		case Img2Tz:
		{
			gFingerSendBuf[10] = Finger_Img2Tz;
			break;
		}

		//存储模板
		//START ADDR PID          LENGTH Command BufferID TempID SUM
		//2      4   1            2      1       1         2      2 字节
		//0xEF01 0xFFFFFFFF 0x01 0x0006  0x06    X         XX     Sum
		case StoreModel:
		{
			gFingerSendBuf[10] = 0x01;
			gFingerSendBuf[11] = 0;
			gFingerSendBuf[12] = gZWbuf[0];
			break;
		}

		//删除固定模板
		//START ADDR PID LENGTH Command TempID TempCount SUM
		//2 字节 4 字节 1 字节 2 字节 1 字节 2 字节 2 字节 2 字节
		//0xEF01 0xFFFFFFFF 0x01 0x0007 0x0c XX XX Sum
		// gZWbuf[0] 要删除的指纹起始地址，gZWbuf[1]要删除的组数
		case DeleteChar:
		{
			gFingerSendBuf[10] = 0;
			gFingerSendBuf[11] = gZWbuf[0];
			gFingerSendBuf[12] = 0;
			gFingerSendBuf[13] = gZWbuf[1];
			break;
		}

		default:
			break;
	}

	for(i = 6; i < len + 9 - 2; i++)
	{
		sum = sum + gFingerSendBuf[i]; //校验和检验
	}

	// 最后两位校验
	gFingerSendBuf[i] = (sum & 0xff00) >> 8;
	gFingerSendBuf[i + 1] = sum;
	RXRcnt = 0;
	SendDataByLen(len + 9);

}

// 返回指纹模块应答信息或FFalse(校验不正确回返回有误）或FALSE(未收到模块应答）
uint8_t UARTReceive_Handle()
{
	uint16_t i, sum = 0;
	uint8_t ok, bufChck1, bufChck2, result;
	ok = UART_Receive_Callback();  // true, 返回有数据， false 返回超时或 起始位不正确

	if(ok == TRUE)
	{
		/////////////////////////////起始码及为响应包////////////////////////////////////////////
		if(gFingerReceiveBuf[0] == 0xEF
		    && gFingerReceiveBuf[1] == 0x01
		    && gFingerReceiveBuf[6] == 0x07)
		{
			///////////////////////校验计算//////////////////////////////////
			for(i = 6; i < gFingerReceiveBuf[8] + 9 - 2; i++)
			{
				sum += gFingerReceiveBuf[i];
			}

			bufChck1 = (sum & 0xff00) >> 8;      //最后两位校验
			bufChck2 = sum;

			if((gFingerReceiveBuf[i] == bufChck1)
			    && (gFingerReceiveBuf[i + 1] == bufChck2))
			{
				result = gFingerReceiveBuf[9];

				switch(result)
				{
					case RET_OK:	// 0xAA（FTRUE）代替0x00（RET_OK）为操作成功以避免误操作 !!(为了理解）
						return FTRUE;

					case RET_NoFinger: // 没有检测到手指
					case RET_TooLowQuality:  // 指纹质量差
					{
						gFingerReceiveBuf[9] = NotGood;
						break;
					}

					case RET_NotIdentified: // 没有找到特征指纹
					{
						gFingerReceiveBuf[9] = FChSi;
						break;
					}

					default: //有返回，数据格式正确，但是没有得到正确答案
					{
						gFingerReceiveBuf[9] = FFalse;
						break;
					}
				}

				return gFingerReceiveBuf[9];
			}
		}
	}

	return FALSE;  //数据格式不正确
}

// 采集图像
uint8_t GetImg_F(void)
{
	uint8_t i, result;

	for(i = 0; i < 100; i++) // 最大采样100次，当其中有一次成立即返回FTRUE，否则返回FALSE
	{
		Send_CMD_Handle(GetImg, 3); // 获取图像
		result = UARTReceive_Handle();

		if(result == FTRUE) // 返回条件（循环出口）
			return FTRUE;
	}

	return FALSE; // 获取图像失败
}

// start ID 为待删除指纹起始ID，count为连续删除指纹个数
uint8_t ShanChZhW(uint8_t startID, uint8_t count)
{
	uint8_t result;
	gZWbuf[0] = startID;
	gZWbuf[1] = count;
	Send_CMD_Handle(DeleteChar, 7);
	result = UARTReceive_Handle();

	if(result != FTRUE)
		return FALSE;

	else
		return FTRUE;
}

void USART2_IRQHandler(void)
{
	uint8_t buf; // 串口接收缓冲区

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		buf = USART_ReceiveData(USART2);
		gFingerReceiveBuf[RXRcnt++] = buf;
	}
}
