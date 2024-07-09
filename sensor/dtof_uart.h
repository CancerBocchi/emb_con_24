#ifndef __DTOF_UART_H__
#define __DTOF_UART_H__

#include <stdio.h>
#include <unistd.h>
#include "ohos_init.h"
#include "cmsis_os2.h"
#include "iot_gpio.h"
#include "iot_uart.h"
#include "hi_io.h"
#include "iot_pwm.h"
#include "hi_pwm.h"


#define DELAY_US            (20000)
#define TOF_RX              (12)
#define TOF_TX              (11)
#define TOF_UART_IDX        (2)

#define DTOF_DATA_LEN       (137)
#define CRC_LEN             (133)

#define IsDTOF_RECIEVE_DONE   (Dtof_Finished_Flag == 1)

extern unsigned char uartReadBuff[300];

extern uint16_t *distance_data[64];

extern unsigned char uartReadBuff[300];
extern unsigned char crcBuff[300];

extern uint16_t *distance_data[64];

extern uint8_t Dtof_Finished_Flag;

void Dtof_Init();

void Dtof_ReadData();

int Dtof_GetPointNum(int min_Dis,int max_Dis);

#endif