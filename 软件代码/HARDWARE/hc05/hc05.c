#include "hc05.h"
#include "delay.h"
#include "usb_ctl.h"
#include <string.h>
#include <stdlib.h>

char buf_arry[64] = {0}; // ���ջ�����
char count = 0; // ����������������¼���յ��ַ�����

/**
 * �������ж�������ַ����Ƿ�Ϊ������
 * ������
 *      str[in] ������ַ���
 *      len[in] ������ַ����ĳ���
 * ����ֵ�������ַ���1�����򷵻�0
 */
static int isNumber(const char *str, int len)
{
	int i = -1;
	char ch;
	while(++i < len)
	{
		ch = str[i];
		if(ch < '0' || ch > '9') // �ж��Ƿ�Ϊ�������ַ�
			return 0;
	}
	return 1;
}


/**
 * ��  ��������3�жϻص��������ڽ��յ��������������ݺ����жϻص��˺���
 * ��  ������
 * ����ֵ����
 */
static void HC05_CallBack(void)
{
	int num  = 0;

	// ��U�̵�Դ����
	if(buf_arry[0] == TurnOff && buf_arry[1] == END_CHAR) // �ж��Ƿ��յ��˽�����־ *#
	{
		printf("TURN OFF\r\n");
		TURN_OFF(); // �ر�U��
	}
	// ��U�̵�Դ����
	else if(isNumber(buf_arry, count)) // �ж��Ƿ�Ϊ������
	{
		num = atoi(buf_arry); // ASCII�ַ���ת����
		if(num <= MAX_NUM) // �ж������Ƿ���Ч����ΧΪ[0-299]
		{
			printf("ID%03d�û�:TURN ON\r\n", num);
			TURN_ON(); // ��U���ϵ�
		}
	}
	// ��ս��ջ�����
	memset(buf_arry, '0', sizeof(buf_arry));
	count = 0;
}


/* ���к������� */

void uart3_init(u32 bound)
{
    // GPIO�˿�����
    GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//ʹ��USART3��GPIOBʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  
	// USART3_TX   GPIOB 10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // USART3_RX	  GPIOB11
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
    GPIO_Init(GPIOB, &GPIO_InitStructure); 

    // Usart2 NVIC ����
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;// ��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		// �����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			// IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	// ����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
    // USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

    USART_Init(USART3, &USART_InitStructure); //��ʼ������3
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
    USART_Cmd(USART3, ENABLE);                    //ʹ�ܴ���3

}

void USART3_SendByte(const uint8_t ch)
{
	USART_SendData(USART3, ch);
	while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
}


// ����3�жϷ�����  1֡���� = [��ʼλ]+[���ݶ�]+[��żУ��]+[����λ]
void USART3_IRQHandler(void)  
{
	uint8_t buf_hc;
    if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) // �����ݵ��˽��ռĴ�����������ɱ�־λUSART_IT_RXNE����λ
    {
        buf_hc = USART_ReceiveData(USART3); // ��ȡһ���ֽ�����
		if(buf_hc != END_CHAR && count < 63)
		{
			buf_arry[count++] = buf_hc; // ���ջ���������СΪ64byte
		}
		else
		{
			buf_arry[count] = END_CHAR; // ���������
			HC05_CallBack(); // ��������жϻص�����
		}
    }
}
