#include "delay.h"
#include "sys.h"
#include "uarttl.h"
#include "usb_ctl.h"
#include "usart.h"
#include "hc05.h"

int main(void)
{
    uint8_t ok_flag = 0;
    delay_init(); // ��ʱ������ʼ��
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    uart1_init(115200); // ���Դ���1
    uart2_init(57600); // ָ��ʶ��ģ��ͨѶ����
    uart3_init(38400); // ����ģ��ͨѶ����
    USBCTL_GPIO_Init(); // U�̵�Դ����IO��

    printf("########################################################\r\n");
    printf("\t\t\tϵͳ�ϵ��ʼ�����!\r\n");
    printf("########################################################\r\n");
    while(1)
    {
        // ֧������ʶ��
        if(!ok_flag) // ָ��ʶ��ɹ���־��TRUE��ʾ�ɹ���FALSE��ʾʧ��
        {
            if(FTRUE == IsValid()) // IsValid������ָ��ͼ��ɼ�����ȡģ�塢�ȶ�ָ��
            {
                ok_flag = TRUE;
            }
        }
        else // ʶ��ɹ�
        {
            ok_flag = FALSE; // ���ʶ��ɹ���־�������´�ʶ��
            printf("ָ��ʶ��ɹ���\r\n"); // ����1����������
            TURN_ON(); // ��U�̵�Դ
        }
    }
}
