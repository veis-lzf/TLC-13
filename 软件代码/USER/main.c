#include "delay.h"
#include "sys.h"
#include "uarttl.h"
#include "usb_ctl.h"
#include "usart.h"
#include "hc05.h"

int main(void)
{
    uint8_t ok_flag = 0;
    delay_init(); // 延时函数初始化
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    uart1_init(115200); // 调试串口1
    uart2_init(57600); // 指纹识别模块通讯串口
    uart3_init(38400); // 蓝牙模块通讯串口
    USBCTL_GPIO_Init(); // U盘电源控制IO口

    printf("########################################################\r\n");
    printf("\t\t\t系统上电初始化完成!\r\n");
    printf("########################################################\r\n");
    while(1)
    {
        // 支持连续识别
        if(!ok_flag) // 指纹识别成功标志，TRUE表示成功，FALSE表示失败
        {
            if(FTRUE == IsValid()) // IsValid：进行指纹图像采集、提取模板、比对指纹
            {
                ok_flag = TRUE;
            }
        }
        else // 识别成功
        {
            ok_flag = FALSE; // 清楚识别成功标志，便于下次识别
            printf("指纹识别成功！\r\n"); // 串口1输出调试语句
            TURN_ON(); // 打开U盘电源
        }
    }
}
