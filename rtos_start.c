/****************************************************************************
 * Afan SAME70 FreeRTOS rtos_start.c
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

#include <atmel_start.h>
#include <peripheral_clk_config.h>

#include "rtos.h"
#include "rtos_start.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char *led_task_name = "led task";
static xTaskHandle  led_thread_id;

/****************************************************************************
 * Private Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/
 
#if configGENERATE_RUN_TIME_STATS
volatile uint32_t ulHighFrequencyTimerTicks;
#endif

/* Prototypes for the standard FreeRTOS callback/hook functions implemented
within this file. */
void vApplicationMallocFailedHook( void );
void vApplicationIdleHook( void );
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName );
void vApplicationTickHook( void );

void vApplicationMallocFailedHook( void )
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeHeapSpace;

	/* This is just a trivial example of an idle hook.  It is called on each
	cycle of the idle task.  It must *NOT* attempt to block.  In this case the
	idle task just queries the amount of FreeRTOS heap that remains.  See the
	memory management section on the http://www.FreeRTOS.org web site for memory
	management options.  If there is a lot of heap memory free then the
	configTOTAL_HEAP_SIZE value in FreeRTOSConfig.h can be reduced to free up
	RAM. */
	xFreeHeapSpace = xPortGetFreeHeapSize();

	/* Remove compiler warning about xFreeHeapSpace being set but never used. */
	( void ) xFreeHeapSpace;
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{

}
/*-----------------------------------------------------------*/

#if configGENERATE_RUN_TIME_STATS
void vConfigureTimerForRunTimeStats(void)
{
  ulHighFrequencyTimerTicks = 0UL;
}
/*-----------------------------------------------------------*/

uint32_t vGetRunTimeCounterValue(void)
{
  return ulHighFrequencyTimerTicks;
}
#endif

/****************************************************************************
 * Name: led_thread_entry
 *
 * Description:
 *   Task for LED.
 *
 * Input Parameters:
 *   NONE.
 *
 * Returned Value:
 *   None.
 *
 ****************************************************************************/

static void led_thread_entry(void *parameter)
{
  (void)parameter;

  while (1)
  {
    printf("led task run\r\n");
    rtos_task_delay_ms(500);
  }
}


/*
 * Example
 */
void FREERTOS_V1000_start(void)
{
  BaseType_t ret;
  ret = xTaskCreate(led_thread_entry, led_task_name,
                    LEDRUN_THREAD_STKSZ, NULL,
					LEDRUN_THREAD_PRIO, &led_thread_id);
  if (ret != pdPASS) {
    printf("%s task with priority-%d create fail\r\n", led_task_name, LEDRUN_THREAD_PRIO);
  }

  vTaskStartScheduler();
}
