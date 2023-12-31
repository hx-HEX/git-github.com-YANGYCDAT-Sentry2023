#include "stm32f4xx_it.h"
#include "user_callback.h"

void SysTick_Handler(void)
{
    User_SysTickCallback();
}

/*************************************************************************
中断处理函数名称：USART1_IRQHandler
中断产生机制：DR16每隔14ms通过DBus发送一帧数据（18字节）,USART1每接收一帧数据
            进入一次中断
函数功能：遥控器传输数据接收
*************************************************************************/
void USART1_IRQHandler(void)
{
	User_UART_RX_Callback(USART1);
}

///*************************************************************************
//中断处理函数名称：USART2_IRQHandler
//中断产生机制：USART2接收到一个空字节后触发中断
//函数功能：裁判系统通讯
//*************************************************************************/
void USART2_IRQHandler(void)
{ 	
    User_UART_RX_Callback(USART2);
}

///*************************************************************************
//中断处理函数名称：USART2_IRQHandler
//中断产生机制：USART2接收到一个空字节后触发中断
//函数功能：裁判系统通讯
//*************************************************************************/
void USART3_IRQHandler(void)
{ 	
    User_UART_RX_Callback(USART3);
}

void UART4_IRQHandler(void)
{
    User_UART_RX_Callback(UART4);
}

void UART5_IRQHandler(void)
{
    User_UART_RX_Callback(UART5);
}

void USART6_IRQHandler(void)
{
    User_UART_RX_Callback(USART6);
}

/*************************************************************************
中断处理函数名称：CAN1_TX_IRQHandler
中断产生机制：
函数功能：
*************************************************************************/
void CAN1_TX_IRQHandler(void)
{

}
/***********************************************************
中断处理函数名称：CAN1_RX0_IRQHandler
中断产生机制：
函数功能：
*************************************************************************/
void CAN1_RX0_IRQHandler(void)
{   
    User_CAN_RX_Callback(CAN1);
}
/*************************************************************************
中断处理函数名称：CAN2_TX_IRQHandler
中断产生机制：
函数功能：
*************************************************************************/
void CAN2_TX_IRQHandler(void)
{

}
/*************************************************************************
中断处理函数名称：CAN2_RX0_IRQHandler
中断产生机制：
函数功能：
*************************************************************************/
void CAN2_RX0_IRQHandler(void)
{
    User_CAN_RX_Callback(CAN2);
}


/*************************************************************************
中断处理函数名称：HardFault_Handler
中断产生机制：
函数功能：硬件错误中断
*************************************************************************/
void HardFault_Handler(void)
{
	while(1);
}
