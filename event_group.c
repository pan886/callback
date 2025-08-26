/**
 * @file main.c
 * @brief

 * @author MCD Application Team
 * @version 1.0
 * @date 2022-03-16
 * @copyright Copyright (c) 2022 Icore, Inc
 */

#include "AiP32RV15A8.h"
#include "FreeRTOS.h" /* Must come first. */
#include "queue.h"    /* RTOS queue related API prototypes. */
#include "semphr.h"   /* Semaphore related API prototypes. */
#include "task.h"     /* RTOS task related API prototypes. */
#include "timers.h"   /* Software timer related API prototypes. */
#include "math.h"
#include <stdio.h>
#include <stdlib.h>
#include "event_groups.h"
 EventBits_t EventValue;
#if 0
GPIO_InitTypeDef GPIOB_struct;
void GPIO_test(void);
uint8_t state = 0;
float abc =0;
void main(void) {
  pll_init();
  sys_io_init();
  uart_init(UART_BOOT_PORT, UART_PARITY_NONE, UART_STOPBITS_1, UART_DATABITS_8,
            UART_BOOT_BD);

  /*Push-pull output mode */
  GPIOB_struct.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIOB_struct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  GPIOB_struct.GPIO_Speed = GPIO_Speed_2MHz;
  /*initialization pin PE2 PE3 PE4 PE5 PE6*/
  GPIO_Init(GPIOB, &GPIOB_struct);

  for (uint32_t i = 0; i < 100; i++) {
    debug("hello world!!!\r\n");
  }

  while (1) {
	 abc=  log(10);
    GPIO_test();
    debug("hello world!\n");
  }
}

void GPIO_Toggle(GPIO_TypeDef *GPIOx, uint16_t PortVal) {
  GPIOx->DATA ^= PortVal;
}
void delay() {
  volatile uint16_t i, j;
  for (i = 0; i < 0xfff; i++)
    for (j = 0; j < 0xff; j++)
      ;
}
void GPIO_test() {

  GPIO_Toggle(GPIOB, GPIO_Pin_13);
  delay();
  GPIO_Toggle(GPIOB, GPIO_Pin_14);
  delay();
  GPIO_Toggle(GPIOB, GPIO_Pin_15);
  delay();
  GPIO_Toggle(GPIOB, GPIO_Pin_15);
  delay();
  GPIO_Toggle(GPIOB, GPIO_Pin_14);
  delay();
  GPIO_Toggle(GPIOB, GPIO_Pin_13);
  delay();
}
#endif

void start_task1(void *pvParameters);
void start_task2(void *pvParameters);
void start_task3(void *pvParameters);

static TaskHandle_t StartTask1_Handler;
static TaskHandle_t StartTask2_Handler;
static TaskHandle_t StartTask3_Handler;

EventGroupHandle_t EventGroupHandler;
SemaphoreHandle_t BinarySemaphore;
SemaphoreHandle_t BinarySemaphore2;
#define EVENTBIT_0   (1<<0)
#define EVENTBIT_1   (1<<1)
#define EVENTBIT_ALL  (EVENTBIT_0|EVENTBIT_1)
GPIO_InitTypeDef GPIOB_struct;
GPIO_InitTypeDef GPIOA_struct;
int key =-1;
typedef enum
{
 ENTER,
 SECLET
}ID_def;
typedef void (*cb)(ID_def key);
cb function;

uint8_t button_flag =0;
void GPIO_Toggle(GPIO_TypeDef *GPIOx, uint16_t PortVal) {
  GPIOx->DATA ^= PortVal;
}

void regist(cb register_function)
{
	function= register_function;
}
EXTI_InitTypeDef EXTI_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;

void PA0_Config(void) {
  /* Enable GPIOA clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

  /* Configure PA.01 pin as output PP */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}


void EXTI0_IRQHandler(void) {
	BaseType_t xHigherPriorityTaskWoken;
  if (EXTI_GetITStatus(EXTI_Line0) != RESET) {
    debug("[IRQ]EXTI Line0 !!!\r\n");
    //xSemaphoreGiveFromISR(BinarySemaphore,&xHigherPriorityTaskWoken);
    xEventGroupSetBitsFromISR(EventGroupHandler,EVENTBIT_0,&xHigherPriorityTaskWoken); /* set led to BLUE */
  //  button_flag =1;

    /* Clear the  EXTI line 0 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line0);
  }
}


void EXTI1_IRQHandler(void) {
	BaseType_t xHigherPriorityTaskWoken;
  if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
    debug("[IRQ]EXTI Line1 !!!\r\n");
   // xSemaphoreGiveFromISR(BinarySemaphore2,&xHigherPriorityTaskWoken);
    xEventGroupSetBitsFromISR(EventGroupHandler,EVENTBIT_1,&xHigherPriorityTaskWoken); /* set led to BLUE */
   // button_flag =1;

    /* Clear the  EXTI line 0 pending bit */
    EXTI_ClearITPendingBit(EXTI_Line1);
  }
}

void EXTI0_Config(void) {
  /* Enable GPIOA clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  /* Configure PA.00 pin as input floating */
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


  /* Enable AFIO clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

  /* Connect EXTI0 Line to PA.00 pin */
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource1);

  /* Configure EXTI0 line */
  EXTI_StructInit(&EXTI_InitStructure);
  EXTI_InitStructure.EXTI_Line = EXTI_Line0|EXTI_Line1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI0 Interrupt to the lowest priority */
  ECLIC_Register_IRQ(EXTI0_IRQn, ECLIC_NON_VECTOR_INTERRUPT,
                     ECLIC_LEVEL_TRIGGER, 2, 0, (void *)EXTI0_IRQHandler);
  /* Enable and set EXTI0 Interrupt to the lowest priority */
  ECLIC_Register_IRQ(EXTI1_IRQn, ECLIC_NON_VECTOR_INTERRUPT,
                     ECLIC_LEVEL_TRIGGER, 2, 0, (void *)EXTI1_IRQHandler);
}

void callback1(ID_def key)
{
	switch(key)
	{
	case ENTER:

		GPIO_Toggle(GPIOB,GPIO_Pin_13);
		break;
	case SECLET:
		GPIO_Toggle(GPIOB,GPIO_Pin_14);
		GPIO_Toggle(GPIOB,GPIO_Pin_15);
		//GPIO_ResetBits(GPIOB,GPIO_Pin_13);
		break;
	}
}


void callback2(ID_def key)
{
	switch(key)
	{
	case ENTER:

		GPIO_Toggle(GPIOB,GPIO_Pin_14);
		break;
	case SECLET:
		GPIO_Toggle(GPIOB,GPIO_Pin_13);
		GPIO_Toggle(GPIOB,GPIO_Pin_14);
		GPIO_Toggle(GPIOB,GPIO_Pin_15);
		break;
	}
}
void main(void) {
  // TimerHandle_t xExampleSoftwareTimer = NULL;
  //sys_io_init();
 // init_uart_prn(0, 0, 0x4); // uart 115200*24, bd 115200
	  pll_init();
	  sys_io_init();
	  uart_init(UART_BOOT_PORT, UART_PARITY_NONE, UART_STOPBITS_1, UART_DATABITS_8,
	            UART_BOOT_BD);

	  PA0_Config();
	  EXTI0_Config();
	  __enable_irq();
	  /*Push-pull output mode */
	  GPIOB_struct.GPIO_Mode = GPIO_Mode_Out_PP;
	  GPIOB_struct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	  GPIOB_struct.GPIO_Speed = GPIO_Speed_2MHz;
	  /*initialization pin PE2 PE3 PE4 PE5 PE6*/
	  GPIO_Init(GPIOB, &GPIOB_struct);

	  BinarySemaphore = xSemaphoreCreateBinary();
	  BinarySemaphore2 = xSemaphoreCreateBinary();
//	    if(BinarySemaphore!=NULL)
//	    	xSemaphoreGive(BinarySemaphore);

  xTaskCreate((TaskFunction_t)start_task1, (const char *)"start_task1",
              (uint16_t)256, (void *)NULL, (UBaseType_t)2, NULL);

  xTaskCreate((TaskFunction_t)start_task2, (const char *)"start_task2",
              (uint16_t)128, (void *)NULL, (UBaseType_t)2, NULL);

  xTaskCreate((TaskFunction_t)start_task3, (const char *)"start_task3",
              (uint16_t)96, (void *)NULL, (UBaseType_t)2, NULL);

  vTaskStartScheduler();
  while (1) {
  }
}

void start_task1(void *pvParameters) {
  int cnt = 0;
  debug("Enter to task_1\r\n");
  EventGroupHandler =xEventGroupCreate();


  while (1) {
    debug("task1 is running %d.....\r\n", cnt++);

//	if(BinarySemaphore!=NULL)
//    	{
//    		if(xSemaphoreTake(BinarySemaphore,portMAX_DELAY)==1)
//    		{
//    			 key = ENTER;
//    			 function(key);
//    		}
//    	}
//    if(BinarySemaphore2!=NULL)
//    		{
//    		if(xSemaphoreTake(BinarySemaphore2,1)==1)
//    		{
//    		    key = SECLET;
//    		    function(key);
//    		    }
//
//
//    	}
 	if(EventGroupHandler != NULL)
    	{


    		EventValue = xEventGroupWaitBits((EventGroupHandle_t)EventGroupHandler,

    				(EventBits_t)EVENTBIT_ALL,(BaseType_t)pdTRUE,(BaseType_t)pdFALSE,(TickType_t)portMAX_DELAY);

          if(EventValue ==0x1)
          {
        	  key = ENTER;

          }

          else if(EventValue ==0x2)
          {
        	  key = SECLET;

          }
          function(key);
    	}



    vTaskDelay(100);
  }
}




void start_task2(void *pvParameters) {
  int cnt = 0;
  debug("Enter to task_2\r\n");
 while (1) {
    debug("task2 is running %d.....\r\n", cnt++);
//    vTaskDelay(200);
//  }
//	if(BinarySemaphore2!=NULL)
//    	{
//    		if(xSemaphoreTake(BinarySemaphore2,portMAX_DELAY)==1)
//    		{
//    			 key = SECLET;
//    			 function(key);
//    		}
	// debug("task2 is running %d.....\r\n", cnt++);
            vTaskDelay(100);
//    			debug("get the BinarySemaphore!\n");
  //  		}
    	}


}

void start_task3(void *pvParameters) {
  int cnt = 0;
  while (1) {
    debug("task3 is running %d.....\r\n", cnt++);
    if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)==1)
   {
   regist(callback2);

     }
     else
  {
 regist(callback1);

      }
    vTaskDelay(100);
  }
}
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
  /* Run time stack overflow checking is performed if
  configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
  function is called if a stack overflow is detected.  pxCurrentTCB can be
  inspected in the debugger if the task name passed into this function is
  corrupt. */
  debug("Stack Overflow\n");
  while (1)
    ;
}

void vApplicationMallocFailedHook(void) {
  /* The malloc failed hook is enabled by setting
  configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

  Called if a call to pvPortMalloc() fails because there is insufficient
  free memory available in the FreeRTOS heap.  pvPortMalloc() is called
  internally by FreeRTOS API functions that create tasks, queues, software
  timers, and semaphores.  The size of the FreeRTOS heap is set by the
  configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
  debug("malloc failed\n");
  while (1)
    ;
}

void vApplicationIdleHook(void)
{}
