#include <stdint.h>
#include <stdbool.h>
#include "TM4C123.h"
#include "FreeRTOS.h"
#include "task.h"
#include "basic_io.h"

// Define LED pin
#define LED_RED   (1U << 1)
#define LED_PORT  GPIOF


TaskHandle_t xLEDBlinkTaskHandle = NULL;
TaskHandle_t xSerialPrintTaskHandle = NULL;


void vLEDBlinkTask(void *pvParameters);
void vSerialPrintTask(void *pvParameters);


void InitGPIO(void)
{
    
    SYSCTL->RCGCGPIO |= (1U << 5);

    
    while ((SYSCTL->PRGPIO & (1U << 5)) == 0) {}

    // Set PF1 as output
    LED_PORT->DIR |= LED_RED;
    LED_PORT->DEN |= LED_RED;
}

/*
int main(void)
{
    // Initialize GPIO for the LED
    InitGPIO();

    // Create the LED blink task
    xTaskCreate(vLEDBlinkTask,       // Task function
                "LED Blink",         // Task name
                100,                 // Stack size
                NULL,                // Parameters
                1,                   // Priority
                &xLEDBlinkTaskHandle // Task handle
    );

    // Create the serial print task
    xTaskCreate(vSerialPrintTask,    // Task function
                "Serial Print",      // Task name
                100,                 // Stack size
                NULL,                // Parameters
                2,                   // Priority
                &xSerialPrintTaskHandle // Task handle
    );

    // Start the scheduler
    vTaskStartScheduler();

    // The program should never reach here
    for (;;);
} */

// LED Blink Task
void vLEDBlinkTask(void *pvParameters)
{
    for (;;)
    {
				// Print a message to the console
        vPrintString("LED Blink Task is running!\n");
			
        // Toggle the RED LED
        LED_PORT->DATA ^= LED_RED;  // Toggle PF1
				
				//vTaskDelay(500); // Delay for 500ms
			
				// Monitor task states
        eTaskState eLEDState1 = eTaskGetState(xLEDBlinkTaskHandle);
        eTaskState eSerialState1 = eTaskGetState(xSerialPrintTaskHandle);

        // Print task states
        vPrintStringAndNumber("LED Blink Task State: ", (unsigned long)eLEDState1);
        vPrintStringAndNumber("Serial Print Task State: ", (unsigned long)eSerialState1);
			
				vTaskDelay(500); // Delay for 500ms
    }
}

// Serial Print Task
void vSerialPrintTask(void *pvParameters)
{
    for (;;)
    {
        // Print a message to the console
        vPrintString("Serial Print Task is running!\n");

        // Monitor task states
        eTaskState eLEDState = eTaskGetState(xLEDBlinkTaskHandle);
        eTaskState eSerialState = eTaskGetState(xSerialPrintTaskHandle);

        // Print task states
        vPrintStringAndNumber("LED Blink Task State: ", (unsigned long)eLEDState);
        vPrintStringAndNumber("Serial Print Task State: ", (unsigned long)eSerialState);

        // Delay for 1 second
        vTaskDelay(1000);
    }
}




/* Idle hook functions MUST be called vApplicationIdleHook(), take no parameters,
and return void. */
void vApplicationIdleHook( void )
{
	// Print a message to the console
  //vPrintString("Idle Hook Task is running!\n");
}
/*-----------------------------------------------------------*/
/*
TaskHandle_t xtask1 = NULL;
TaskHandle_t xtask2 = NULL;

void vTask1(void *pvParameters);
void vTask2(void *pvParameters);

int main(void)
{
    InitGPIO();
    
    xTaskCreate(vTask1, "task 1", 100, NULL, 2, &xtask1);  
    xTaskCreate(vTask2, "task 2", 100, NULL, 2, &xtask2);  
    
    vTaskStartScheduler();
    
    // Should never reach here if scheduler started successfully
    for(;;);
    
    return 0;
}

void vTask1(void *pvParameters)
{
    TickType_t lastWakeTime;
    lastWakeTime = xTaskGetTickCount();
    
    while(1)
    {
      // Toggle the red LED
      vPrintString("Task1 is running!\n");  
			LED_PORT->DATA ^= LED_RED;  
        
			// Monitor task states
      eTaskState etask1State = eTaskGetState(xtask1);
      eTaskState etask2State = eTaskGetState(xtask2);
			
      vTaskDelayUntil(&lastWakeTime, 350 / portTICK_PERIOD_MS);
    }
}

void vTask2(void *pvParameters)
{
    while(1)
    {
      vPrintString("Task2 is running!\n");
      LED_PORT->DATA ^= LED_RED;
        
			// Monitor task states
      eTaskState etask1State = eTaskGetState(xtask1);
      eTaskState etask2State = eTaskGetState(xtask2);
			
      // Simple delay, not guaranteed to be periodic
      vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}
*/








#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "basic_io.h"

TaskHandle_t xfirstTaskHandle, xsecondTaskHandle, xthirdTaskHandle;
SemaphoreHandle_t xBinSemaphore, xMutex1, xMutex2;

void vFirstTask(void *pvParameters)
{
    vPrintString("first task taking mutex1\n");
    xSemaphoreTake(xMutex1, portMAX_DELAY);
    vPrintString("first task took mutex1\n");

    vTaskDelay(pdMS_TO_TICKS(500));

    vPrintString("first task taking mutex2\n");
    xSemaphoreTake(xMutex2, portMAX_DELAY);
    vPrintString("first task took mutex2\n");

		vPrintString("you will never get here \n");

    vTaskDelete(NULL);
}

void vSecTask(void *pvParameters)
{
		vPrintString("second task taking mutex2\n");
    xSemaphoreTake(xMutex2, portMAX_DELAY);
    vPrintString("second task took mutex2\n");

    vTaskDelay(pdMS_TO_TICKS(500));

    vPrintString("second task taking mutex1\n");
    xSemaphoreTake(xMutex1, portMAX_DELAY);
    vPrintString("second task took mutex1\n");

		vPrintString("you will never get here \n");

    vTaskDelete(NULL);
}

void vStalker(void *pvParameters)
{
    while (1)
    {
        vPrintString("I'm still waiting by the way!\n");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

int main(void)
{

	  xMutex1 = xSemaphoreCreateMutex();
    xMutex2 = xSemaphoreCreateMutex();

    xTaskCreate(vFirstTask, "first task", 200, NULL, 2, NULL);
    xTaskCreate(vSecTask, "second task", 200, NULL, 2, NULL);
		xTaskCreate(vStalker, "Stalker", 200, NULL, 1, NULL);

    vTaskStartScheduler();

    while (1);
}
