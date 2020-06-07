#include "uarttl.h"
#include "delay.h"

// ȫ�ֱ���
uint8_t gFingerSendBuf[20] = {0, 0, 0, 0, 0, 0, 0, 0}; // ָ��ʶ��ģ��ͨѶ�� �������������ݰ�����󳤶�[20 byte]
uint8_t gFingerReceiveBuf[20] = {0, 0, 0, 0, 0, 0, 0, 0}; // ������Ӧ���ݰ�������[20 byte]
uint8_t gZWbuf[3] = {0, 0, 0}; // ָ������ID������
uint8_t Finger_Img2Tz = 1; // ָ������ģ�壨1�� 2��
uint8_t RXRcnt = 0; // �������������ڽ�����Ӧ���ݰ�����

/**
 * ��������ɣ�1.ָ��ͼ��Ĳɼ���2.��ȡ����ģ��1��3.�ȶ�ָ��
 * ��������
 * ����ֵ��ʶ��ɹ�����FTRUE(0xAA)����֮����FALSE(0)
 */
uint8_t IsValid(void)
{
	uint8_t result;

	result = GetImg_F(); // ��ȡͼ��

	if(!result)
		return FALSE; // ��ȡͼ��ʧ��

	Finger_Img2Tz = 1; // ��ȡ������ģ��1
	Send_CMD_Handle(Img2Tz, 4);
	result = UARTReceive_Handle();

	if(result != FTRUE)
		return FALSE;

	Send_CMD_Handle(Search, 8); // ����ָ��
	result = UARTReceive_Handle();

	if(result != FTRUE)
		return FALSE;

	return FTRUE;
}

// ���ָ�ƽ��պͷ��ͻ���������������������0
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

// ���ڽ��ջص�����
// ���ã���ģ�鷵�ص�����������ջ�����
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
			// �յ���ʼ�����ֽھ͵ȹ̶���
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

		return TRUE;   //�ɹ���������
	}

	else
	{
		return FALSE;
	}

}

/* ���к������� */
// ���ڳ�ʼ��
void uart2_init(u32 bound)
{
	// GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	//ʹ��USART2��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	// USART2_TX   GPIOA2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// USART2_RX	  GPIOA3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//PA3
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Usart2 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3 ; // ��ռ���ȼ�3
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

	USART_Init(USART2, &USART_InitStructure); //��ʼ������2
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
	USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ���2

}

// ����2����һ���ֽ�����
void USART2_SendByte(const uint8_t ch)
{
	USART_SendData(USART2, ch);

	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
}

// �����������ݰ�
void SendDataByLen(uint8_t len)
{
	uint8_t i;

	for(i = 0; i < len; i++)
	{
		USART2_SendByte(gFingerSendBuf[i]);
	}
}

// �ϳ������������ݰ�
// 1.�ɼ�ͼ���������ݰ�(GetImg)12 byte
// 2.ͼ��ת�����������ݰ�(Img2TZ)13 byte
// 3.�������ݿ�ȶ�ָ���������ݰ�(Search) 17 byte
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
	gFingerSendBuf[6] = 0x01; // ָ��
	gFingerSendBuf[7] = 0x00; // 7��8����������ݳ���
	gFingerSendBuf[8] = len;
	gFingerSendBuf[9] = CMD;

	switch(CMD)
	{
		// �����ݿ�������
		// START ADDR PID LENGTH Command BufferID TempID TempCount SUM
		// 2 �ֽ� 4 �ֽ� 1 �ֽ� 2 �ֽ� 1 �ֽ� 1 �ֽ� 2 �ֽ� 2 �ֽ� 2 �ֽ�
		// 0xEF01 0xFFFFFFFF 0x01 0x0008 0x04 X XX XX Sum
		// ����ָ��
		case Search:
		{
			gFingerSendBuf[10] = 0x01;
			gFingerSendBuf[11] = 0x00;
			gFingerSendBuf[12] = 0x00;
			gFingerSendBuf[13] = 0x01;
			gFingerSendBuf[14] = 0x2C;
			break;
		}

		//��ȡ������ID0/ID1
		case Img2Tz:
		{
			gFingerSendBuf[10] = Finger_Img2Tz;
			break;
		}

		//�洢ģ��
		//START ADDR PID          LENGTH Command BufferID TempID SUM
		//2      4   1            2      1       1         2      2 �ֽ�
		//0xEF01 0xFFFFFFFF 0x01 0x0006  0x06    X         XX     Sum
		case StoreModel:
		{
			gFingerSendBuf[10] = 0x01;
			gFingerSendBuf[11] = 0;
			gFingerSendBuf[12] = gZWbuf[0];
			break;
		}

		//ɾ���̶�ģ��
		//START ADDR PID LENGTH Command TempID TempCount SUM
		//2 �ֽ� 4 �ֽ� 1 �ֽ� 2 �ֽ� 1 �ֽ� 2 �ֽ� 2 �ֽ� 2 �ֽ�
		//0xEF01 0xFFFFFFFF 0x01 0x0007 0x0c XX XX Sum
		// gZWbuf[0] Ҫɾ����ָ����ʼ��ַ��gZWbuf[1]Ҫɾ��������
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
		sum = sum + gFingerSendBuf[i]; //У��ͼ���
	}

	// �����λУ��
	gFingerSendBuf[i] = (sum & 0xff00) >> 8;
	gFingerSendBuf[i + 1] = sum;
	RXRcnt = 0;
	SendDataByLen(len + 9);

}

// ����ָ��ģ��Ӧ����Ϣ��FFalse(У�鲻��ȷ�ط������󣩻�FALSE(δ�յ�ģ��Ӧ��
uint8_t UARTReceive_Handle()
{
	uint16_t i, sum = 0;
	uint8_t ok, bufChck1, bufChck2, result;
	ok = UART_Receive_Callback();  // true, ���������ݣ� false ���س�ʱ�� ��ʼλ����ȷ

	if(ok == TRUE)
	{
		/////////////////////////////��ʼ�뼰Ϊ��Ӧ��////////////////////////////////////////////
		if(gFingerReceiveBuf[0] == 0xEF
		    && gFingerReceiveBuf[1] == 0x01
		    && gFingerReceiveBuf[6] == 0x07)
		{
			///////////////////////У�����//////////////////////////////////
			for(i = 6; i < gFingerReceiveBuf[8] + 9 - 2; i++)
			{
				sum += gFingerReceiveBuf[i];
			}

			bufChck1 = (sum & 0xff00) >> 8;      //�����λУ��
			bufChck2 = sum;

			if((gFingerReceiveBuf[i] == bufChck1)
			    && (gFingerReceiveBuf[i + 1] == bufChck2))
			{
				result = gFingerReceiveBuf[9];

				switch(result)
				{
					case RET_OK:	// 0xAA��FTRUE������0x00��RET_OK��Ϊ�����ɹ��Ա�������� !!(Ϊ����⣩
						return FTRUE;

					case RET_NoFinger: // û�м�⵽��ָ
					case RET_TooLowQuality:  // ָ��������
					{
						gFingerReceiveBuf[9] = NotGood;
						break;
					}

					case RET_NotIdentified: // û���ҵ�����ָ��
					{
						gFingerReceiveBuf[9] = FChSi;
						break;
					}

					default: //�з��أ����ݸ�ʽ��ȷ������û�еõ���ȷ��
					{
						gFingerReceiveBuf[9] = FFalse;
						break;
					}
				}

				return gFingerReceiveBuf[9];
			}
		}
	}

	return FALSE;  //���ݸ�ʽ����ȷ
}

// �ɼ�ͼ��
uint8_t GetImg_F(void)
{
	uint8_t i, result;

	for(i = 0; i < 100; i++) // ������100�Σ���������һ�γ���������FTRUE�����򷵻�FALSE
	{
		Send_CMD_Handle(GetImg, 3); // ��ȡͼ��
		result = UARTReceive_Handle();

		if(result == FTRUE) // ����������ѭ�����ڣ�
			return FTRUE;
	}

	return FALSE; // ��ȡͼ��ʧ��
}

// start ID Ϊ��ɾ��ָ����ʼID��countΪ����ɾ��ָ�Ƹ���
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
	uint8_t buf; // ���ڽ��ջ�����

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		buf = USART_ReceiveData(USART2);
		gFingerReceiveBuf[RXRcnt++] = buf;
	}
}
