#ifndef __HC05_H
#define __HC05_H
#include "stdio.h"	
#include "sys.h" 

#define MAX_NUM 299
#define END_CHAR '#'

enum EType_Status
{
	TurnOff = '*'
};

void uart3_init(u32 bound);
void USART3_SendByte(const uint8_t ch);

#endif
