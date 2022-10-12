/*
 * FreeRTOS V202112.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "lpc21xx.h"
#include "bit_math.h"

/* Drivers includes */
#include "GPIO.h"
#include "serial.h"

/*-----------------------------------------------------------*/
/* Macros for code Implementation */
/*-----------------------------------------------------------*/
#define configButton_USE_RC_Filter  1  //Debouncing is performed using physical RC filter

/* Constants to setup UART1 */
#define mainTX_ENABLE		( ( unsigned long ) 0x00010000 )	/* UART1. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )


/* Defining Tasks ID for the reciver identification */
#define button1ID        (('1')<<1)
#define button2ID        (('2')<<1)
#define periodicID       (('3')<<1)


/* Global Variables Definition*/
xQueueHandle SimpleQueue;      //Queue handelr used to inter-process comunication

// Handlers for created tasks
TaskHandle_t xHandleButton1 = NULL;
TaskHandle_t xHandleButton2 = NULL;
TaskHandle_t xHandlePeriodic = NULL;
TaskHandle_t xHandleRecevier = NULL;
TaskHandle_t xHandleLoad1 = NULL;
TaskHandle_t xHandleLoad2 = NULL;
TaskHandle_t xHandleStatus = NULL;  // This is for Run time analysis 
char runTimeStats[290];

/* Timers values for tracing tasks execution*/
uint32_t task_1_in=0,task_1_out=0,task_1_total_time =0;  // Load 1 Task time holders
uint32_t task_2_in=0,task_2_out=0,task_2_total_time =0;  // Load 2 Task time holders
uint32_t task_3_in=0,task_3_out=0,task_3_total_time =0;  // Receiver Task time holders
uint32_t task_4_in=0,task_4_out=0,task_4_total_time =0;  // Periodic Task time holders
uint32_t task_5_in=0,task_5_out=0,task_5_total_time =0;  // Button 1 Monitor Task time holders
uint32_t task_6_in=0,task_6_out=0,task_6_total_time =0;  // Button 2 Monitor Task time holders
uint32_t task_7_in=0,task_7_out=0,task_7_total_time =0;  // IDLE Task time holders
uint32_t system_time=0;
float cpu_load=0, task_load_1=0,task_load_2=0,task_receiver=0,task_periodic=0,task_button_1=0,task_button_2=0,task_idle=0; 

/* Inserting the definition of xTaskPeriodicCreate() as only i can send main.c, tasks.c, and FreeRTOSConfig.h */
extern BaseType_t xTaskPeriodicCreate( TaskFunction_t pxTaskCode,
                            const char * const pcName,
                            const configSTACK_DEPTH_TYPE usStackDepth,
                            void * const pvParameters,
                            UBaseType_t uxPriority,
														TickType_t period,
                            TaskHandle_t * const pxCreatedTask );

														
/* Structure Definition to pass to two instance of vButton_Monitor() to defrentiate both logics */														
typedef struct
{
	int id;                // Is the id of the sending task ie: Button 1 or 2
	pinX_t pin;            // Pin number in the GPIO Register
}	buttonParam_t;

void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Hardware Initialization funtion */
static void prvSetupHardware( void )
{
	/* Confguring the GPIO pins */
	GPIO_init();	
	CLR_BIT(IODIR1, PIN1);   //Re-configuring the input GPIO Port1.Pin1 as input just incase..
	CLR_BIT(IODIR1, PIN2);   //Re-configuring the input GPIO Port1.Pin2 as input just incase..
  /* Configure the UART1 pins.  All other pins remain at their default of 0. */
	PINSEL0 |= mainTX_ENABLE;
  /* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);
	/* Config trace timer 1 and read T1TC to get current tick */
	T1PR = 10;
	T1TCR |= 0x1;
/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}


////////////////////////////////////////////////////////////////////////////////////
//  Tasks                                                                         //
////////////////////////////////////////////////////////////////////////////////////

// Button monitoring Task with periodicity 20ms, and Execution time 20us
void xTaskReceiver( void * pvParameters )
{    
	TickType_t xLastWakeTime = xTaskGetTickCount();
	int received=0;
    for( ;; )
    {
			if (xQueueReceive(SimpleQueue, &received, 0) != pdTRUE)
		{
					
		}
		else
		{
			switch (received)
			{
				case (button1ID):
					vSerialPutString((const signed char *)"Button 1 is relesed\n",20);
				break;
				case (button2ID):
					vSerialPutString((const signed char *)"Button 2 is relesed\n",20);
				break;
				case ((button1ID)|PIN_IS_HIGH):
					vSerialPutString((const signed char *)"Button 1 is pressed\n",20);
				break;
				case ((button2ID)|PIN_IS_HIGH):
					vSerialPutString((const signed char *)"Button 2 is pressed\n",20);
				break;
				case (periodicID):
					vSerialPutString((const signed char *)"Periodic Task Message\n",22);
				break;
			}
		}
		vTaskDelayUntil( &xLastWakeTime, 20 );
    }
}

// Periodic Task with periodicity 100ms, and Execution time 20us
void xPeriodicTask( void * pvParameters )
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
		static int i = periodicID;
    for( ;; )
    {
			xQueueSend(SimpleQueue,&i , portMAX_DELAY);
			vTaskDelayUntil( &xLastWakeTime, 100 );
    }
}

// Button monitoring task the which well be create two instance to monitore both buttons
// Both instance get the required pin to monitor from pvParameters passed from main function
// Button monitoring Task with periodicity 50ms, and Execution time 20us
void xButtonMonitorTask (void * pvParameters )
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	buttonParam_t * pin = (buttonParam_t *) pvParameters;
	int id = (pin->id);
	pinState_t buttonSwitch;
	unsigned char oldState=PIN_IS_LOW; //variable to monitor user action

	#if (configButton_USE_RC_Filter ==0)
	unsigned char count =0; //simple algorithm for Debouncing
#endif

	for ( ;; )
	{
		buttonSwitch = GPIO_read(PORT_1,pin->pin);

#if (configButton_USE_RC_Filter ==0)
		
		if( buttonSwitch != oldState)
		{
			
			count++;
			
			if(count >=3) //Ensure that the button is pressed continuoly for 30 ms
			{
				id = (pin->id)|buttonSwitch;
				//xQueueSend(SimpleQueue,&(pin)->id , portMAX_DELAY);
				xQueueSend(SimpleQueue,&id , portMAX_DELAY);
				oldState = buttonSwitch;				
			}
		}
		else
		{
			count =0;
		}

#else

		if( buttonSwitch != oldState)
		{
			
			id = (pin->id)|buttonSwitch;
		  xQueueSend(SimpleQueue,&id , portMAX_DELAY);
	  	oldState = buttonSwitch;				
			
		}
		
#endif
		vTaskDelayUntil( &xLastWakeTime, 50 );
	}
}

// CPU Load Task with periodicity 10ms, and Execution time 5ms
void Load_1_Task( void * pvParameters )
{
		TickType_t xLastWakeTime = xTaskGetTickCount();
		volatile int i = 2;
    for( ;; )
    {
			for(i=0;i<5145;i++)
			{
				GPIO_write(PORT_0,PIN15,PIN_IS_LOW);
			}
			vTaskDelayUntil( &xLastWakeTime, 10 );
			xLastWakeTime = xTaskGetTickCount();
    }
}

// CPU Load Task with periodicity 100ms, and Execution time 12ms
void Load_2_Task( void * pvParameters )
{
		TickType_t xLastWakeTime = xTaskGetTickCount();
		volatile int i = 2;
    for( ;; )
    {
			for(i=0;i<12000;i++) 
			{
				GPIO_write(PORT_0,PIN15,PIN_IS_HIGH);
			}
			vTaskDelayUntil( &xLastWakeTime, 100 );
			xLastWakeTime = xTaskGetTickCount();
    }
}

void runTime_status( void * pvParameters )
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
    for( ;; )
    {
			vTaskDelayUntil( &xLastWakeTime, 120 );
			if(system_time < T1TC)
			{
			system_time = T1TC;
			task_load_1 = (float)(task_1_total_time *100)/system_time;
			task_load_2 = (float)(task_2_total_time *100)/system_time;
			task_receiver = (float)(task_3_total_time *100)/system_time;
			task_periodic = (float)(task_4_total_time *100)/system_time;
			task_button_1 = (float)(task_5_total_time *100)/system_time;
			task_button_2 = (float)(task_6_total_time *100)/system_time;
			task_idle = (float)(task_7_total_time *100)/system_time;
			cpu_load = 100 - task_idle;
			}
			else // If the Timer 1 counter T1TC overflowed
			{
			system_time = task_load_1 = task_load_2 =	task_receiver =	task_periodic = task_button_1 = task_button_2 = task_idle = cpu_load = 0;
				task_1_total_time =  task_2_total_time = task_3_total_time = task_4_total_time = task_5_total_time = task_6_total_time = system_time= task_7_total_time=0;
			}
			
			
    }
}

/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
  
static buttonParam_t button_1 = {button1ID,PIN1};
static buttonParam_t button_2 = {button2ID,PIN2};
int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();

#if (configUSE_EDF_SCHEDULER ==0)	
  xTaskCreate(xTaskReceiver      ,"Reciver" ,100 ,NULL      ,3 ,&xHandleRecevier );    	
	xTaskCreate(xPeriodicTask      ,"Period"  ,100 ,NULL      ,2 ,&xHandlePeriodic );    	
	xTaskCreate(Load_1_Task        ,"Load1"   ,100 ,NULL      ,2 ,&xHandleLoad1 );   
	xTaskCreate(Load_2_Task        ,"Load2"   ,100 ,NULL      ,1 ,&xHandleLoad2 );   
	xTaskCreate(xButtonMonitorTask ,"Button1" ,100 ,&button_1 ,1 ,&xHandleButton1 );    	
	xTaskCreate(xButtonMonitorTask ,"Button2" ,100 ,&button_2 ,1 ,&xHandleButton2 );
#else	
	xTaskPeriodicCreate(Load_1_Task       ,"Load1"   ,100 ,NULL      ,0 ,10  ,&xHandleLoad1 );  
	xTaskPeriodicCreate(Load_2_Task       ,"Load2"   ,100 ,NULL      ,0 ,100 ,&xHandleLoad2 );   		    	
	xTaskPeriodicCreate(xTaskReceiver     ,"Receiver" ,100 ,NULL      ,0 ,20  ,&xHandleRecevier );  
	xTaskPeriodicCreate(xPeriodicTask     ,"Period"  ,100 ,NULL      ,0 ,100 ,&xHandlePeriodic);    		
	xTaskPeriodicCreate(xButtonMonitorTask,"Button1" ,100 ,&button_1 ,0 ,50  ,&xHandleButton1 );  
	xTaskPeriodicCreate(xButtonMonitorTask,"Button2" ,100 ,&button_2 ,0 ,50  ,&xHandleButton2 );    		
	xTaskPeriodicCreate(runTime_status    ,"Status"  ,100 ,NULL      ,0 ,500 ,&xHandleStatus );    		
#endif
	
#if (configUSE_APPLICATION_TASK_TAG ==1)
	vTaskSetApplicationTaskTag( xHandleLoad1    , ( TaskHookFunction_t ) '1' );
	vTaskSetApplicationTaskTag( xHandleLoad2    , ( TaskHookFunction_t ) '2' );
	vTaskSetApplicationTaskTag( xHandleRecevier , ( TaskHookFunction_t ) '3' );
	vTaskSetApplicationTaskTag( xHandlePeriodic , ( TaskHookFunction_t ) '4' );
	vTaskSetApplicationTaskTag( xHandleButton1  , ( TaskHookFunction_t ) '5' );
	vTaskSetApplicationTaskTag( xHandleButton2  , ( TaskHookFunction_t ) '6' );
	vTaskSetApplicationTaskTag( xHandleStatus   , ( TaskHookFunction_t ) 'S' );
#endif

	SimpleQueue = xQueueCreate(5, sizeof (int));
  if (SimpleQueue == 0)  // Queue not created
  {
		vSerialPutString((const signed char *)"Unable to create Integer Queue\n", 31);
  }
  else
  {
	  vSerialPutString((const signed char *)"Integer Queue Created successfully\n\n", 35);
  }

	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	GPIO_write(PORT_0,PIN9,PIN_IS_HIGH);
	GPIO_write(PORT_0,PIN9,PIN_IS_LOW);
}
/*-----------------------------------------------------------*/

