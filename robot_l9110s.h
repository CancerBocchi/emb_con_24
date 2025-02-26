/*
 * Copyright (c) 2022 HiSilicon (Shanghai) Technologies CO., LIMITED.
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
 * limitations under the License.
 */

#ifndef ROBOT_L9110S_H
#define ROBOT_L9110S_H
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <memory.h>
#include "compile_define.h"
#include "iot_gpio.h"
#include "iot_pwm.h"

void init_car_drive(void);
void car_backward(uint32_t pwm_value);
void car_drive(int left_or_right , int value);
void car_drive_diff(int pwm_value, int diff);
void car_stop(void);

#endif
