#ifndef __HEADFILE_H__
#define __HEADFILE_H__

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <hi_stdlib.h>
#include "iot_gpio_ex.h"
#include "compile_define.h" //define encoder pinport
#include "cmsis_os2.h"
#include "ohos_init.h"
#include "hi_io.h"
#include "iot_uart.h"

#include "car.h"
#include "robot_l9110s.h"   //pwm car drive
#include "mpu6050.h"
#include "wheel_codec.h"
#include "PID.h"
#include "coordinate.h"
#include "map.h"
#include "dtof_uart.h"

#include "a_star.h"
#include "avoid.h"
#include "coordinate.h"
#include "state.h"
#include "udp_com.h"

extern int avoid_flag;


#endif