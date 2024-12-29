#include "stm32f4xx_hal.h"
#include "USART3.h"
#include "string.h"
unsigned char Uart6ReceiveBuf[300] = {0};
uint8_t databuf[20]={0};

//  结构体定义
USARTDATA Uart3;
USARTDATA Uart6;
UART_HandleTypeDef huart6;
/**********************************************************************************************************
函数名称：UART3配置
输入参数：无
输出参数：无
函数返回：无
**********************************************************************************************************/
// USART3_TX	 PB10	//  out
// USART3_RX	 PB11	//  in
//void UART3_Configuration(void)
//{
//    GPIO_InitTypeDef GPIO_InitStructure;
//    USART_InitTypeDef USART_InitStructure;
//
//    //  开启GPIO_D的时钟
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
//
//    //  开启串口3的时钟
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
//
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
//    GPIO_Init(GPIOC, &GPIO_InitStructure);
//
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//    GPIO_Init(GPIOC, &GPIO_InitStructure);
//
//
//    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
//    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);
//
//
//    USART_InitStructure.USART_BaudRate = 115200;
//    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//    USART_InitStructure.USART_StopBits = USART_StopBits_1;
//    USART_InitStructure.USART_Parity = USART_Parity_No;
//    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
//
//    USART_Init(USART3, &USART_InitStructure);
//
//    /* 使能串口3 */
//    USART_Cmd(USART3, ENABLE);
//    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
//}
void UART3_Configuration(void)
{
    /* 定义GPIO初始化结构体 */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* 定义USART初始化结构体 */
    /* 定义UART句柄 */
    UART_HandleTypeDef huart3;

    __HAL_RCC_GPIOC_CLK_ENABLE();  // 开启GPIOC的时钟
    __HAL_RCC_USART3_CLK_ENABLE(); // 开启串口3的时钟

    // 配置PC10 (USART3_TX) 和 PC11 (USART3_RX) 为复用推挽输出，上拉，2MHz速度
    GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // 对应2MHz
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3; // 设置为USART3复用功能
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // 初始化USART3
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        // Initialization Error
        Error_Handler();
    }

    // 使能接收中断
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
}



///**********************************************************************************************************
//函数名称：putchar函数重定义
//输入参数：无
//输出参数：无
//函数返回：无
//**********************************************************************************************************/
//int fputc(int ch, FILE *f)
//{
//    USART3->SR;                                                         // 防止复位后无法打印首字符
//
//    USART_SendData(USART3, (uint8_t) ch);
//    while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
//    { ;
//    }
//
//    return (ch);
//}
//
///**********************************************************************************************************
//函数名称：USART3发送数据函数
//输入参数：发送数据首地址和数据长度
//输出参数：无
//**********************************************************************************************************/
//void USART3_Senddata(unsigned char *Data, unsigned int length)
//{
//    while (length--)
//    {
//        USART_SendData(USART3, *Data++);
//        while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
//    }
//}

/**********************************************************************************************************
函数名称：UART1配置
输入参数：无
输出参数：无
函数返回：无
**********************************************************************************************************/
// USART6_TX	 PC6	//  out
// USART6_RX	 PC7	//  in
//void UART6_Configuration(unsigned int baud)
//{
//    GPIO_InitTypeDef GPIO_InitStructure;
//    USART_InitTypeDef USART_InitStructure;
//    NVIC_InitTypeDef NVIC_InitStructure;
//
//    Uart6.ReceiveFinish = 0;
//    Uart6.RXlenth = 0;
//    Uart6.Time = 0;
//    Uart6.Rxbuf = Uart6ReceiveBuf;
//
//    //  开启GPIOA的时钟
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
//
//    //  开启串口1的时钟
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
//
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
//    GPIO_Init(GPIOC, &GPIO_InitStructure);
//
//    GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
//    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);
//
//
//    USART_InitStructure.USART_BaudRate = baud;
//    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//    USART_InitStructure.USART_StopBits = USART_StopBits_1;
//    USART_InitStructure.USART_Parity = USART_Parity_No;
//    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
//
//    USART_Init(USART6, &USART_InitStructure);
//
//    /* 使能串口1 */
//    USART_Cmd(USART6, ENABLE);
//    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
//
//    /* NVIC configuration */
//    /* Configure the Priority Group to 2 bits */
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//
//    /* Enable the USARTx Interrupt */
//    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);
//
//    /* Enable USART */
//    USART_Cmd(USART6, ENABLE);
//}
void UART6_Configuration(unsigned int baud)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();  // 开启GPIOC的时钟
    __HAL_RCC_USART6_CLK_ENABLE(); // 开启串口6的时钟

    // 初始化Uart6结构体成员
    Uart6.ReceiveFinish = 0;
    Uart6.RXlenth = 0;
    Uart6.Time = 0;
    Uart6.Rxbuf = Uart6ReceiveBuf;

    // 配置PC6 (USART6_TX) 和 PC7 (USART6_RX) 为复用推挽输出，上拉，2MHz速度
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // 对应2MHz
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6; // 设置为USART6复用功能
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // 初始化USART6
    huart6.Instance = USART6;
    huart6.Init.BaudRate = baud;
    huart6.Init.WordLength = UART_WORDLENGTH_8B;
    huart6.Init.StopBits = UART_STOPBITS_1;
    huart6.Init.Parity = UART_PARITY_NONE;
    huart6.Init.Mode = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart6) != HAL_OK)
    {
        // Initialization Error
        Error_Handler();
    }

    // 使能接收中断
//    __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);


    // 使能USART6中断
//    HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
//    HAL_NVIC_EnableIRQ(USART6_IRQn);

    // 使能USART6
    __HAL_UART_ENABLE(&huart6);
}
/**********************************************************************************************************
函数名称：USART6发送数据函数
输入参数：发送数据首地址和数据长度
输出参数：无
**********************************************************************************************************/
void USART6_Senddata(unsigned char *Data, unsigned int length)
{
    HAL_UART_Transmit(&huart6, Data, length, 0xFFFF);
}

void USART6_IRQHandler(void)
{
//    if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)                //  若接收数据寄存器满
//    {
//        USART_ClearITPendingBit(USART6, USART_IT_RXNE);
//
//        Uart6.Rxbuf[Uart6.RXlenth] = USART_ReceiveData(USART6);
//
//        if (Uart6.RXlenth == 0 && Uart6.Rxbuf[0] != 0x55)
//        {
//            Uart6.RXlenth = 0;
//            return;
//        }
//        if (Uart6.RXlenth == 1 && Uart6.Rxbuf[1] != 0x53)
//        {
//            Uart6.RXlenth = 0;
//            return;
//        }
//
//
//        Uart6.RXlenth++;
//
//        if(Uart6.RXlenth == 11)
//        {
//            memcpy(databuf, &Uart6.Rxbuf[0], 20);
//
//            display_flag = 1;
//            Uart6.RXlenth = 0;
//        }
//    }


        // 确保只处理USART6的回调

        if (__HAL_UART_GET_IT_SOURCE(&huart6, UART_IT_RXNE) != RESET)  // 如果接收数据寄存器满
        {
            __HAL_UART_CLEAR_FLAG(&huart6, UART_IT_RXNE);
            uint8_t data = (uint8_t)HAL_UART_Receive(&huart6, NULL, 0, 0); // 清除RXNE标志位

            // 读取接收到的数据
            data = (uint8_t)HAL_UART_Receive(&huart6, &data, 1, HAL_MAX_DELAY);

            Uart6.Rxbuf[Uart6.RXlenth] = data;

            // 检查起始字节
            if (Uart6.RXlenth == 0 && data != 0x55)
            {
                Uart6.RXlenth = 0;
                return;
            }
            if (Uart6.RXlenth == 1 && data != 0x53)
            {
                Uart6.RXlenth = 0;
                return;
            }

            Uart6.RXlenth++;

            // 如果接收到完整的包（假设长度为11）
            if (Uart6.RXlenth == 11)
            {
                memcpy(databuf, Uart6.Rxbuf, 11);  // 注意：原代码中memcpy的大小是20，但RXlenth只有11

//                display_flag = 1;
                Uart6.RXlenth = 0;
            }
        }
}
