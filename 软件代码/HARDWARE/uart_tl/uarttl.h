#ifndef __UARTTL_H
#define __UARTTL_H
#include "stdio.h"
#include "sys.h"

extern uint8_t gFingerSendBuf[20];
extern uint8_t gFingerReceiveBuf[20];
extern uint8_t gZWbuf[3];
extern uint8_t Finger_Img2Tz;
extern uint16_t flag;
extern uint8_t buf;

typedef enum _BOOL_
{
	FALSE,
	TRUE
} BOOL;

enum
{
	FTRUE = 0xAA,		// 指纹模块返回操作ok
	FFalse = 0xff,
	FChSi = 0xee, 		// 采样超时
	NotGood = 0xdd, 	// 图像质量差
	ExistFinger = 0xbb, // 指纹存在
};

typedef enum _CMDCODE
{
	GetImg = 0x01,
	Img2Tz,
	Match,
	Search,
	RegModel,
	StoreModel,
	DeleteChar = 0x0c,
	Empty = 0x0d

} CMDCODE;

typedef enum _RESULT_CODE
{
	RET_OK,
	RET_NoFinger = 0x02,
	RET_TooLowQuality = 0x06,
	RET_TooFewPoint,
	RET_NotMatched,
	RET_NotIdentified
} RESULT_CODE;

void uart2_init(u32 bound);
void USART2_SendByte(const uint8_t ch);
uint8_t GetImg_F(void);
uint8_t ShanChZhW(uint8_t startID, uint8_t count);
uint8_t UARTReceive_Handle(void);
void Send_CMD_Handle(uint8_t CMD, uint8_t len);
uint8_t IsValid(void);

#endif
