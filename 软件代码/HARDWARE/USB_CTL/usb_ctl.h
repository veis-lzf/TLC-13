#ifndef __USB_CTL_H
#define __USB_CTL_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/9/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

#define USB_CTL 	PAout(1)
#define TURN_OFF() 	GPIO_SetBits(GPIOA, GPIO_Pin_1)
#define TURN_ON() 	GPIO_ResetBits(GPIOA, GPIO_Pin_1)

void USBCTL_GPIO_Init(void);//��ʼ��

		 				    
#endif
