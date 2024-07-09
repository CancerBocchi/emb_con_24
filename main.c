/*
 * Copyright (c) 2020 Nanjing Xiaoxiongpai Intelligent Technology Co., Ltd.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "headfile.h"

#define THREAD_STACK_SIZE (1024 * 8)
#define THREAD_PRIO 25
#define THREAD_DELAY_1S 2000000
#define THREAD_DELAY_1S 2000000
#define THREAD_DELAY_500MS 500000
#define WIFI_IOT_UART_IDX_1 1
#define WIFI_IOT_UART_IDX_2 2
#define UART_BUFF_SIZE 8

static int car_init_flag = 0;
static int dtof_init_flag = 0;
static int udp_init_flag = 0;
/**
 * @brief encoder task
 *
 */
void MotorTask(void)
{   
    printf("MotorTask entry\n");
    car_init();
    car_init_flag = 1;
    while (1){
        car_run();
    }
}

static void PatrolTask(void){

    while(car_init_flag == 0||dtof_init_flag == 0){
        osDelay(10);
    };
    Axis_patrol_reset();
    State_Machine_Init();
    // Axis_set_map(map1);
    // Axis_set_patrolNode(patrol_2,4);
    while(1){
        // Axis_patrol_run();

        State_Machine();
    }
}


/**
 * @brief UartTask
 *
 */
int avoid_flag;
static void UartTask(void)
{
    while(udp_init_flag == 0){
        osDelay(10);
    };
    printf("uart_task\n");
    Dtof_Init();
    dtof_init_flag = 1;
    while(1){
        Dtof_ReadData();
        int num = Dtof_GetPointNum(0,300);
        // printf("point_num between 0 and 200 is %d\n",num);
        if(num >=50)
            avoid_flag = 1;
        else
            avoid_flag = 0;

        printf("dtof:num:%d\n",num);
        
        memset(uartReadBuff,0,sizeof(uartReadBuff));
        // num = Dtof_GetPointNum(0,300);
        // printf("point_num between 0 and 300 is %d\n",num);
        // num = Dtof_GetPointNum(0,400);
        // printf("point_num between 0 and 400 is %d\n",num);
    }
    
    // printf("UartTask entry\n");
    // uint8_t uart_buff_1[UART_BUFF_SIZE] = {0};
    // uint8_t *uart_buff_ptr_1 = uart_buff_1;
    // uint8_t ret;
    // hi_io_set_func(HI_IO_NAME_GPIO_11, HI_IO_FUNC_GPIO_11_UART2_TXD); /* uart2 tx */
    // hi_io_set_func(HI_IO_NAME_GPIO_12, HI_IO_FUNC_GPIO_12_UART2_RXD); /* uart2 rx */
    
    // IotUartAttribute uart_attr = {

    //     // baud_rate: 115200
    //     .baudRate = 115200, 

    //     // data_bits: 8bits
    //     .dataBits = 8,
    //     .stopBits = 1,
    //     .parity = 0,
    // };
    // // // Initialize uart driver
    // ret = IoTUartInit(WIFI_IOT_UART_IDX_2, &uart_attr);
    // if (ret != IOT_SUCCESS)
    // {
    //     printf("Failed to init uart2! Err code = %d\n", ret);
    //     return;
    // }

    // while (1)
    // {

    //     // send data through uart1
    //     IoTUartRead(WIFI_IOT_UART_IDX_2, uart_buff_ptr_1, UART_BUFF_SIZE);
    //     if (uart_buff_ptr_1[0] != 0)
    //     {
    //         if (!strcmp(uart_buff_ptr_1, "1"))
    //         {
    //             Axis_move_to_target_position(0,0);
    //         }
    //         else if (!strcmp(uart_buff_ptr_1, "2"))
    //         {
    //             Axis_move_to_target_position(10,0);
    //         }
    //         else if (!strcmp(uart_buff_ptr_1, "3"))
    //         {
    //             Axis_move_to_target_position(10,10);
    //         }
    //         else if (!strcmp(uart_buff_ptr_1, "4"))
    //         {
    //             printf("car_rotate\n");
    //            Car_Change_Yaw(180);
    //         }
    //         else if (!strcmp(uart_buff_ptr_1, "5"))
    //         {
    //             printf("car_rotate_move1");
    //             Car_Change_Yaw(360);
    //         }
    //         else if (!strcmp(uart_buff_ptr_1, "6"))
    //         {
    //             printf("car_rotate_move2");
    //             Car_Change_Yaw(60);
    //         }
    //         else if(uart_buff_ptr_1[0] == 'P'){
    //             float p = atof(uart_buff_ptr_1+1);
    //             printf("p:%f\n",p);
    //             angle_con.Kp = p;
    //         }
    //         else if(uart_buff_ptr_1[0] == 'D'){
    //             float d = atof(uart_buff_ptr_1+1);
    //             printf("d:%f\n",d);
    //             angle_con.Kd = d;
    //         }
    //         else
    //             printf("command not found!\n");
    //         memset(uart_buff_ptr_1, 0, 8);
    //     }
    //     else
    //         ;
    //     usleep(40000);
    // }
}

static void AvoidTask(){
    while(1){
        osSemaphoreAcquire(avoid_sem,osWaitForever);

            avoid_run();

        osSemaphoreRelease(avoid_sem);
        
    }

}



#include "lwip/sockets.h"
#include "wifi_connect.h"

#define CONFIG_WIFI_SSID "BearPi"    // 要连接的WiFi 热点账号
#define CONFIG_WIFI_PWD "20030323yzj"  // 要连接的WiFi 热点密码
#define CONFIG_CLIENT_PORT 8888      // 要连接的服务器端口
#define TCP_BACKLOG 10

char udp_recvbuf[512];
char *buf = "Hello! I'm BearPi-HM_Nano UDP Server!";

char udp_sendbuf[128];
int s1=2;

static void UDPServerTask(void)
{
    // 在sock_fd 进行监听，在 new_fd 接收新的链接
    int sock_fd, new_fd;

    // 服务端地址信息
    struct sockaddr_in server_sock;

    // 客户端地址信息
    struct sockaddr_in client_sock, *cli_addr;
    int sin_size;

    // 连接Wifi
    WifiConnect(CONFIG_WIFI_SSID, CONFIG_WIFI_PWD);

    // 创建socket
    if ((sock_fd = socket(AF_INET, SOCK_DGRAM , 0)) == -1) {
        perror("socket is error\r\n");
        return;
    }

    bzero(&server_sock, sizeof(server_sock));
    server_sock.sin_family = AF_INET;
    server_sock.sin_addr.s_addr = htonl(INADDR_ANY);
    // server_sock.sin_addr.s_addr = inet_addr(CONFIG_SERVER_IP);
    
    server_sock.sin_port = htons(CONFIG_CLIENT_PORT);

    // 调用bind函数绑定socket和地址
    if (bind(sock_fd, (struct sockaddr *)&server_sock, sizeof(struct sockaddr)) == -1) {
        return;
    }

    // // 调用listen函数监听(指定port监听)
    // if (listen(sock_fd, TCP_BACKLOG) == -1) {
    //     return;
    // }

    printf("start accept\n");
    udp_init_flag = 1;

    while(car_init_flag == 0){
        osDelay(10);
    };


    // 调用accept函数从队列中
    while (1) {
        sin_size = sizeof(struct sockaddr_in);

        // 处理目标
        ssize_t ret;

        while (1) {
            memset_s(udp_recvbuf, sizeof(udp_recvbuf), 0, sizeof(udp_recvbuf));
            if ((ret = recvfrom(sock_fd, udp_recvbuf, sizeof(udp_recvbuf), 0 ,(struct sockaddr *)&client_sock, (socklen_t *)&sin_size )) == -1) {
                printf("recv error \r\n");
            }   
            State_copy_buffer(udp_recvbuf);
            State_command_deal();
            // printf("recv :%s\r\n", udp_recvbuf);
            // sleep(TASK_DELAY_2S);
            // udp_sendbuf[0] = current_x;
            // udp_sendbuf[1] = current_y;
            // static int a = 0;
            // static int b = 0;
            // a++;
            // b++;
            
            sprintf(udp_sendbuf, "%d,%d",current_x,current_y); //产生"123"
            if ((ret = sendto(sock_fd, udp_sendbuf, strlen(udp_sendbuf) + 1, 0 ,(struct sockaddr *)&client_sock , sizeof(client_sock) )) == -1) {
                perror("send : ");
            }
            s1++;
            // sleep(TASK_DELAY_2S);
        }
        close(new_fd);
    }
}

/**
 * @brief Main Entry of the Thread Example
 *
 */
static void ThreadExample(void)
{
    osThreadAttr_t attr;

    attr.name = "MotorTask";
    attr.attr_bits = 0U;
    attr.cb_mem = NULL;
    attr.cb_size = 0U;
    attr.stack_mem = NULL;
    attr.stack_size = THREAD_STACK_SIZE;
    attr.priority = 24;

    //用于控制车辆
    if (osThreadNew((osTimerFunc_t)MotorTask, NULL, &attr) == NULL)
    {
        printf("Failed to create MotorTask!\n");
    }

    //用于和激光雷达通信
    attr.name = "UartTask";
    attr.priority = 26;
    if (osThreadNew((osTimerFunc_t)UartTask, NULL, &attr) == NULL)
    {
        printf("Failed to create UartTask!\n");
    }
    //车主要任务运行函数
    attr.name = "PatrolTask";
    attr.priority = 24;
    attr.stack_size = THREAD_STACK_SIZE*2;
    if (osThreadNew((osTimerFunc_t)PatrolTask, NULL, &attr) == NULL)
    {
        printf("Failed to create PatrolTask!\n");
    }
    //避障信号量
    avoid_sem = osSemaphoreNew(4,0,NULL);
    if(avoid_sem == NULL){
        printf("Failed to create avoid_sem!\n");
    }
    //用于避障
    attr.name = "AvoidTask";
    attr.stack_size = THREAD_STACK_SIZE;
    attr.priority = 25;
    if (osThreadNew((osTimerFunc_t)AvoidTask, NULL, &attr) == NULL)
    {
        printf("Failed to create AvoidTask!\n");
    }
    //用于UDP通信
    attr.name = "UDPServerTask";
    attr.priority = 25;
    if (osThreadNew((osThreadFunc_t)UDPServerTask, NULL, &attr) == NULL) {
        printf("[UDPServerDemo] Failed to create UDPServerTask!\n");
    }

    IoTWatchDogDisable();
}

APP_FEATURE_INIT(ThreadExample);
