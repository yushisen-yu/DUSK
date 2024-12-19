//
// Created by fairy on 2024/10/7.
//

#include "fsmc.h"

#ifdef USE_FSMC

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"//内嵌了stm32f4xx_ll_fsmc.h


SRAM_HandleTypeDef hsram1;//TFT模块
DMA_HandleTypeDef hdma_memtomem_dma2_stream6;
/** FSMC GPIO Configuration
PE7   ------> FSMC_D4
PE8   ------> FSMC_D5
PE9   ------> FSMC_D6
PE10   ------> FSMC_D7
PE11   ------> FSMC_D8
PE12   ------> FSMC_D9
PE13   ------> FSMC_D10
PE14   ------> FSMC_D11
PE15   ------> FSMC_D12
PD8   ------> FSMC_D13
PD9   ------> FSMC_D14
PD10   ------> FSMC_D15
PD11   ------> FSMC_A16
PD14   ------> FSMC_D0
PD15   ------> FSMC_D1
PD0   ------> FSMC_D2
PD1   ------> FSMC_D3
PD4   ------> FSMC_NOE
PD5   ------> FSMC_NWE
PD7   ------> FSMC_NE1  //这个与实际情况有出入，应该是PB 7
*/

// FSMC配置
void fsmc_init()
{
    //	使能PD,PE,PF,PG时钟
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    /*使能FSMC时钟*/
    __HAL_RCC_FSMC_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    /**************
    *GPIO配置
    *************/
    GPIO_InitTypeDef GPIO_InitStructure;

    //	LCD_BL
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStructure.Pin = GPIO_PIN_9;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;                        //	复用输出	    //	推挽输出
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;                    //	100MHz
    GPIO_InitStructure.Pull = GPIO_PULLDOWN;                        //	上拉
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);


    //	PD0,1,4,5,8,9,10,14,15 AF OUT
    GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8
                             | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;                    //	100MHz
    GPIO_InitStructure.Pull = GPIO_PULLUP;                        //	上拉
    GPIO_InitStructure.Alternate = GPIO_AF12_FSMC;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStructure);                                //	初始化

    //	PE7~15,AF OUT
    GPIO_InitStructure.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11
                             | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    //复用加推挽
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;                       //复用加推挽
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;                    //	100MHz
    GPIO_InitStructure.Pull = GPIO_PULLUP;                        //	上拉
    HAL_GPIO_Init(GPIOE, &GPIO_InitStructure);                                //	初始化

    //	LCD_RS PG1,FSMC_A11
    GPIO_InitStructure.Pin = GPIO_PIN_1;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;                    //	100MHz
    GPIO_InitStructure.Pull = GPIO_PULLUP;                        //	上拉
    HAL_GPIO_Init(GPIOG, &GPIO_InitStructure);                                //	初始化

    //	LCD_CS PG12,FSMC_NE4
    GPIO_InitStructure.Pin = GPIO_PIN_12;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;                    //	100MHz
    GPIO_InitStructure.Pull = GPIO_PULLUP;                        //	上拉
    HAL_GPIO_Init(GPIOG, &GPIO_InitStructure);                                //	初始化



    // FSMC时序
    FMC_NORSRAM_TimingTypeDef readWriteTiming;
    FMC_NORSRAM_TimingTypeDef writeTiming;

    readWriteTiming.AddressSetupTime = 0x0F;//	地址建立时间（ADDSET）为16个HCLK 1/168M=6ns*16=96ns
    readWriteTiming.AddressHoldTime = 0x00;//	地址保持时间（ADDHLD）模式A未用到
    readWriteTiming.DataSetupTime = 60;    //	数据保存时间为60个HCLK	=6*60=360ns
    readWriteTiming.BusTurnAroundDuration = 0x00;
    readWriteTiming.CLKDivision = 0x00;
    readWriteTiming.DataLatency = 0x00;
    readWriteTiming.AccessMode = FSMC_ACCESS_MODE_A; //	模式A


    writeTiming.AddressSetupTime = 9;        //	地址建立时间（ADDSET）为9个HCLK =54ns
    writeTiming.AddressHoldTime = 0x00;             //	地址保持时间（A
    writeTiming.DataSetupTime = 8;      //	数据保存时间为6ns*9个HCLK=54ns
    writeTiming.BusTurnAroundDuration = 0x00;
    writeTiming.CLKDivision = 0x00;
    writeTiming.DataLatency = 0x00;
    writeTiming.AccessMode = FSMC_ACCESS_MODE_A;     //	模式A

    hsram1.Init.NSBank = FSMC_NORSRAM_BANK4; //  这里我们使用NE4 ，也就对应BTCR[6],[7]。
    hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;// 	不复用数据地址
    hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_NOR;
    hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;   // 16位数据宽度
    hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;  // 是否使能突发访问,仅对同步突发存储器有效,此处未用到
    hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;// 等待信号的极性,仅在突发模式访问下有
    hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;     // 存储器是在等待周期之前的一个时钟周期还是等待周期期间使能NWAIT
    hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;             // 等待使能位,此处未用到
    hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
    hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;     // 存储器写使能
    hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;// 是否使能同步传输模式下的等待信号,此处未用
    hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_ENABLE;        // 	读写使用不同的时序
    hsram1.Init.WriteBurst = FSMC_WRITE_BURST_ENABLE;            // 开启突发写，性能翻倍
    HAL_SRAM_Init(&hsram1, &readWriteTiming, &writeTiming);
}


/*启用FSMC+DMA向LCD传输数据*/
#ifdef USE_FSMC_DMA

void fsmc_dma_init()
{
    __HAL_RCC_DMA2_CLK_ENABLE();

    hdma_memtomem_dma2_stream6.Instance = DMA2_Stream6;
    hdma_memtomem_dma2_stream6.Init.Channel = DMA_CHANNEL_0;//只要是DMA请求表中空白的部分就可以为自己所用？
    hdma_memtomem_dma2_stream6.Init.Direction = DMA_MEMORY_TO_MEMORY;
    hdma_memtomem_dma2_stream6.Init.PeriphInc = DMA_PINC_ENABLE;//源地址自增
    hdma_memtomem_dma2_stream6.Init.MemInc = DMA_MINC_DISABLE;  //目的地址不自增，因为LCD的地址是固定的
    hdma_memtomem_dma2_stream6.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_memtomem_dma2_stream6.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_memtomem_dma2_stream6.Init.Mode = DMA_NORMAL;//传输模式，此即传输一次停止
    hdma_memtomem_dma2_stream6.Init.Priority = DMA_PRIORITY_VERY_HIGH;
    hdma_memtomem_dma2_stream6.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma_memtomem_dma2_stream6.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_memtomem_dma2_stream6.Init.MemBurst = DMA_MBURST_INC8;   //源地址,16太快了，否则屏幕无反应
    hdma_memtomem_dma2_stream6.Init.PeriphBurst = DMA_MBURST_INC8;//目的地址,16太快了，否则白屏
    if (HAL_DMA_Init(&hdma_memtomem_dma2_stream6) != HAL_OK)
    {
        Error_Handler();
    }
    /*不受FreeRTOS调度*/
    HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
    /*注册回调函数
    * 流程：开启DMA中断后，DMA开始传输数据，传输完之后回到【pCallback】，需要注意的是中断处理函数得要定义*/
//    HAL_DMA_RegisterCallback(&hdma_memtomem_dma2_stream6, HAL_DMA_XFER_CPLT_CB_ID, (void(*)(DMA_HandleTypeDef *))pCallback);
}

///**
// * @brief DMA中断
//*/
//// 主打一个狂野
//#define DMA2_S6CR (*((volatile uint32_t *)0x400264A0))
//// DMA中断,里面他宝贝的真啰嗦
//void DMA2_Stream6_IRQHandler(void)
//{
//    HAL_DMA_IRQHandler(&hdma_memtomem_dma2_stream6);
//
//    /*  检查传输完成标志（TCIF）是否被设置，即是否传输完成标志位*/
////    if (DMA2->HISR & 0x1 << 21)
////    {
////        DMA2->HISR &= ~(0x1 << 21);// 清除标志位
////        // 检查中断使能标志位
////        if (DMA2_S6CR & (DMA_IT_TC))
////        {
////            DMA2_S6CR &= ~(DMA_IT_TC);// 清除标志位
////            /* Change the DMA state */
////            hdma_memtomem_dma2_stream6.State = HAL_DMA_STATE_READY;// 不能少,因为Start_IT里需要靠它来开启
////            /* Process Unlocked */
////            __HAL_UNLOCK(&hdma_memtomem_dma2_stream6);// 不能少
////        }
////        //
////        LVGL_LCD_FSMC_DMA_pCallback();
////    }
//}


#endif

#endif