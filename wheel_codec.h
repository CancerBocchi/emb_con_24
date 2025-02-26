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

#ifndef WHEEL_CODEC_H
#define WHEEL_CODEC_H

#include <iot_gpio.h>
#include <iot_errno.h>
#include <stdint.h>

extern void init_wheel_codec(void);
extern int16_t get_wheel_cnt_left(void);
extern int16_t get_wheel_cnt_right(void);
extern void get_wheel_cnt(int16_t *left, int16_t *right);
extern int get_encoder_left();
extern int get_encoder_right();

#endif

