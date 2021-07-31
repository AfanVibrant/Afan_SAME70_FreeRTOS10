/****************************************************************************
 * Afan SAME70 FreeRTOS port/rtos.h
 * Author : Afan Vibrant (AfanVibrant@outlook.com)
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef PORT_RTOS_H
#define PORT_RTOS_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>
#include <timers.h>

#include <stdio.h>
#include <stdbool.h>
#include <compiler.h>

#include "rtos_config.h"

/* Very crude mechanism used to determine if the critical
   section handling functions are being called from an interrupt
   context or not.  This relies on the interrupt handler setting
   this variable manually.
 */
static volatile BaseType_t xInsideISR = pdFALSE;

#define RTOS_TICK_PER_SECOND      configTICK_RATE_HZ
#define RTOS_USEC_PER_TICK        (1000000/RTOS_TICK_PER_SECOND)

#define rtos_irq_enter()          xInsideISR = pdTRUE
#define rtos_irq_exit()           xInsideISR = pdFALSE

#define rtos_get_tickcount()      xTaskGetTickCount()
#define rtos_get_tick_ms()        (1000UL*xTaskGetTickCount())/configTICK_RATE_HZ

#define rtos_enter_critical()     portENTER_CRITICAL()
#define rtos_exit_critical()      portEXIT_CRITICAL()

typedef xSemaphoreHandle          rtos_sem_t;
typedef xSemaphoreHandle          rtos_mutex_t;
typedef xQueueHandle              rtos_mbox_t;
typedef xTimerHandle              rtos_timer_t;
typedef int                       rtos_thread_t;
typedef unsigned int             rtos_prot_t;

#define rtos_task_delay(n)        vTaskDelay(n)
#define rtos_task_delay_ms(n)     rtos_task_delay(1000*n/RTOS_USEC_PER_TICK)
#define rtos_task_delay_us(n)     uint32_t tick = (n > RTOS_USEC_PER_TICK) ? \
                                                  (n + 1)/RTOS_USEC_PER_TICK : 1; \
                                   rtos_task_delay(tick)

#define RTOS_MBOX_NULL            (rtos_mbox_t)0
#define RTOS_SEM_NULL             (rtos_sem_t)0

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PORT_RTOS_H */
