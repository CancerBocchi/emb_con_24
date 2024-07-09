/*
 * Copyright (C) 2021 HiHope Open Source Organization .
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 *
 * limitations under the License.
 */

#include <stdio.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio.h"
#include "iot_uart.h"
#include "hi_io.h"
#include "iot_pwm.h"
#include "hi_pwm.h"

#include "dtof_uart.h"

unsigned char uartReadBuff[300] = {0};
unsigned char crcBuff[300] = {0};
static unsigned int crc32table[256];

uint16_t *distance_data[64];

uint8_t Dtof_Finished_Flag = 0;

/*
 * 初始化uart1接口配置
 * Initialize uart1 interface configuration
 */
int usr_uart_config(void)
{
    IotUartAttribute g_uart_cfg = {115200, 8, 1, IOT_UART_PARITY_NONE, 500, 500, 0};
    int ret = IoTUartInit(TOF_UART_IDX, &g_uart_cfg);
    if (ret != 0) 
    {
        printf("uart init fail\r\n");
    }

    return ret;
}


static void init_crc_table(void)
{
    unsigned int c;
    unsigned int i, j;
    for (i = 0; i < 256; i++) 
    {
        c = (unsigned int)i;
        for (j = 0; j < 8; j++) 
        {
            if (c & 1)
                c = 0xedb88320L ^ (c >> 1);
            else
                c = c >> 1;
        }                                                                                                                                            
        crc32table[i] = c;
    }
};

/*
 * crc校验
 * crc checking
 */

static unsigned int crc32(const unsigned char *buf, unsigned int size)
{
    unsigned int  i, crc = 0xFFFFFFFF;

    for (i = 0; i < size; i++) 
    {
        crc = crc32table[(crc ^ buf[i]) & 0xff] ^ (crc >> 8); /* 8: 右移8bit */
    }
    return crc ^ 0xFFFFFFFF;
}

/*
 * 配置uart1、pwm0、红灯、黄灯、绿灯的gpio管脚
 * Configure GPIO pins for uart1, wm0, red light, yellow light, and green light
 */
static void gpio_init(void)
{
    IoTGpioInit(TOF_TX);  
    hi_io_set_func(TOF_TX, HI_IO_FUNC_GPIO_11_UART2_TXD);
    IoTGpioInit(TOF_RX);  
    hi_io_set_func(TOF_RX, HI_IO_FUNC_GPIO_12_UART2_RXD);
}

/**
 * @brief 初始化通信 IO 以及串口
 * 
 */
void Dtof_Init(){
    //初始化串口引脚
    gpio_init();    
    //初始化串口
    usr_uart_config();
    //初始化串口
    init_crc_table();
}

/**
 * @brief 通过串口得到 DTOF的数据，并且将数据存储在对应的数组里
 * 
 */
void Dtof_ReadData(){

    static int d = 0; 
    short int i = 0;

    unsigned int len = 0; //接收到的串口数据长度
    unsigned int crc_data = 0; //接收CRC校验后的值
    while(1){
        //一次接收数据量要足够
        len += IoTUartRead(TOF_UART_IDX, uartReadBuff+len, DTOF_DATA_LEN);
        if (len >= DTOF_DATA_LEN) {
            if(uartReadBuff[0] == 0xAA && uartReadBuff[1] == 0x55 && uartReadBuff[132] == 0xFF) {
                memcpy(crcBuff,uartReadBuff,CRC_LEN);
                crc_data = crc32(crcBuff,CRC_LEN);  
                //CRC校验
                if(crc_data == *(unsigned int *)(&uartReadBuff[CRC_LEN])) {
                    // printf("info get successfully!\n");
                    //读取距离信息
                    for(i = 0; i < 64; i++){
                        distance_data[i] = (unsigned short int *)(&uartReadBuff[i*2+4]);
                        // printf("%d:%d\n",i,*distance_data[i]);
                    } 
                        
                }
                else{
                    printf("crc32 fail !!!\r\n");
                }
            }
            else
                printf("dtof:error\n");
                
            len = 0;
            crc_data = 0;
            memset(crcBuff,0,sizeof(crcBuff));
            break;
        }
    }
    Dtof_Finished_Flag = 1;
    usleep(DELAY_US);
}

/**
 * @brief 统计区间内的点的个数
 * 
 * @param min_Dis 
 * @param max_Dis 
 */
int Dtof_GetPointNum(int min_Dis,int max_Dis){
    int num;
    for(int i = 0;i<64;i++){
        //printf("%d:%d\n",i,*distance_data[i]);
        if((*distance_data[i])<max_Dis&&(*distance_data[i])>min_Dis){
            num++;
        }
    }
    return num;
}
