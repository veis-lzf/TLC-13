#include "hc05.h"
#include "delay.h"
#include "usb_ctl.h"
#include <string.h>
#include <stdlib.h>

char buf_arry[64] = {0}; // 接收缓冲区
char count = 0; // 计数变量，用来记录接收的字符个数

/**
 * 描述：判断输入的字符串是否为纯数字
 * 参数：
 *      str[in] 待检测字符串
 *      len[in] 待检测字符串的长度
 * 返回值：纯数字返回1，否则返回0
 */
static int isNumber(const char *str, int len)
{
	int i = -1;
	char ch;
	while(++i < len)
	{
		ch = str[i];
		if(ch < '0' || ch > '9') // 判断是否为非数字字符
			return 0;
	}
	return 1;
}


/**
 * 描  述：串口3中断回调函数，在接收到蓝牙发来的数据后，在中断回调此函数
 * 参  数：无
 * 返回值：无
 */
static void HC05_CallBack(void)
{
	int num  = 0;

	// 关U盘电源操作
	if(buf_arry[0] == TurnOff && buf_arry[1] == END_CHAR) // 判断是否收到了结束标志 *#
	{
		printf("TURN OFF\r\n");
		TURN_OFF(); // 关闭U盘
	}
	// 开U盘电源操作
	else if(isNumber(buf_arry, count)) // 判断是否为纯数字
	{
		num = atoi(buf_arry); // ASCII字符串转整数
		if(num <= MAX_NUM) // 判断数字是否有效，范围为[0-299]
		{
			printf("ID%03d用户:TURN ON\r\n", num);
			TURN_ON(); // 给U盘上电
		}
	}
	// 清空接收缓冲区
	memset(buf_arry, '0', sizeof(buf_arry));
	count = 0;
}


/* 公有函数区域 */

void uart3_init(u32 bound)
{
    // GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//使能USART3，GPIOB时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  
	// USART3_TX   GPIOB 10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // USART3_RX	  GPIOB11
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOB, &GPIO_InitStructure); 

    // Usart2 NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;// 抢占优先级3
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

    USART_Init(USART3, &USART_InitStructure); //初始化串口3
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启串口接受中断
    USART_Cmd(USART3, ENABLE);                    //使能串口3

}

void USART3_SendByte(const uint8_t ch)
{
	USART_SendData(USART3, ch);
	while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
}


// 串口3中断服务函数  1帧数据 = [起始位]+[数据段]+[奇偶校验]+[结束位]
void USART3_IRQHandler(void)  
{
	uint8_t buf_hc;
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) // 有数据到了接收寄存器，接收完成标志位USART_IT_RXNE被置位
    {
        buf_hc = USART_ReceiveData(USART3); // 读取一个字节数据
		if(buf_hc != END_CHAR && count < 63)
		{
			buf_arry[count++] = buf_hc; // 接收缓冲区，大小为64byte
		}
		else
		{
			buf_arry[count] = END_CHAR; // 存入结束符
			HC05_CallBack(); // 进入接收中断回调函数
		}
    }
}
