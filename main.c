#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>
#include "queue.h"
#include "semphr.h"
#include "ports_init.h"
#include "lcd.h"

#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/pin_map.h"
#include "driverlib/debug.h"
#include "driverlib/uart.h"
#include "driverlib/timer.h"


/* Global Variables */
#define DPri						2
#define PPri						1
#define DC_Motor_Enable GPIO_PIN_1  																					//pf1 --> motor enable
#define buzzen 					GPIO_PIN_0																						//pd0 --> buzzer
#define ENC_PIN_A    		GPIO_PIN_6   																					//pd6 --> encoder's first pin
#define ENC_PIN_B    		GPIO_PIN_7   																					//pd7 --> encoder's second pin
									
#define WINDOW_OPEN  		HIGH																										
#define WINDOW_CLOSED  	LOW																											
#define MIDDLE					2
#define UP							HIGH																										
#define DOWN						LOW																											

#define STOP						0																											//stop
#define PU							1																											//passenger up
#define PD							2																											//passenger down
#define DU							3																											//driver up
#define DD							4																											//driver down

#define ENC_MIN_POS  0    																										// Minimum encoder position
#define ENC_MAX_POS  10   																										// Maximum encoder position
#define ENC_DEBOUNCE_MS 2 																										// Reduced debounce time for better responsiveness

static volatile int32_t encoderPos = 5;             													// starts in middle of range
static volatile uint8_t lastEncoderState = 0;       													// last encoder state
static volatile uint32_t lastEncoderTime = 0;       													// for debouncing
static volatile int32_t encoderDirection = 0;       													// direction tracking for debugging



bool driver_elevate_button_state;																							
bool driver_lower_button_state;																								
bool passenger_elevate_button_state;																					
bool passenger_lower_button_state;																						
bool lock_state;																															
bool ir_sensor_state;																													
bool upper_limit_switch_state;																								
bool lower_limit_switch_state;																								
int	 window_state = MIDDLE;																										
bool operation;																																
int last_task = STOP;


TaskHandle_t 				xDriverWindowElevateTaskHandle 									= NULL;				
TaskHandle_t 				xDriverWindowLowerTaskHandle 										= NULL;				
TaskHandle_t 				xPassengerWindowElevateTaskHandle 							= NULL;				
TaskHandle_t 				xPassengerWindowLowerTaskHandle 								= NULL;				
TaskHandle_t 				xObsatcleDetectionHandle 												= NULL;				
TaskHandle_t 				xLockWindowsTaskHandle		 											= NULL;				
TaskHandle_t				xUpperLimitActionHandle													= NULL;				
TaskHandle_t				xLowerLimitActionHandle													= NULL;				
TaskHandle_t 				xEncoderTaskHandle = NULL;				
SemaphoreHandle_t 	xDriverWindowElevateTaskUnlockerSemaphore 			= NULL;				
SemaphoreHandle_t 	xDriverWindowLowerTaskUnlockerSemaphore 				= NULL;				
SemaphoreHandle_t 	xPassengerWindowElevateTaskUnlockerSemaphore		= NULL;				
SemaphoreHandle_t 	xPassengerWindowLowerTaskUnlockerSemaphore			= NULL;				
SemaphoreHandle_t 	xLockWindowsTaskUnlockerSemaphore 							= NULL;				
SemaphoreHandle_t 	xObstacleDetectionUnlockerSemaphore 						= NULL;				
SemaphoreHandle_t		xUpperLimitUnlockerSemaphore										=	NULL;				
SemaphoreHandle_t		xLowerLimitUnlockerSemaphore										=	NULL;				


void vDUpTask						(void *pvParameters);
void vDLowerTask				(void *pvParameters);
void vPUpTask						(void *pvParameters);
void vPLowerTask				(void *pvParameters);
void vLockWindows				(void *pvParameters);
void vODetection				(void *pvParameters);
void vUpperLimit				(void *pvParameters);
void vEncoderTask				(void *pvParameters);
void vLowerLimit				(void *pvParameters);
void ISRHandlers				(void);

static inline uint8_t readAB(void)
{
    // PD6?bit6, PD7?bit7
    uint8_t a = (GPIO_PORTD_DATA_R >> 6) & 1;
    uint8_t b = (GPIO_PORTD_DATA_R >> 7) & 1;
    return (a << 1) | b;  // 0..3
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//MAIN

int main()
{
    //port init
    PortA_Config();                                                                                                    
    PortC_Config();   
		PortD_Config();
    PortF_Config();                                                                                                     
    
    //initialize lcd
    LCD_Init();                                                                                                       
    LCD_SetBacklight(1);											//turn on backlight                                                                                        
    LCD_Clear();                              //clear display                                                     
    LCD_SetCursor(0, 0);											//write in first line
    LCD_Print("Power Window Sys");						
    LCD_SetCursor(1, 0);											//write in second line
    LCD_Print("Initializing...");
    
    for(volatile int i = 0; i < 1000000; i++);
    
    //initialize interrupts
    GPIOIntRegister    (GPIO_PORTA_BASE, ISRHandlers);                                                               
    GPIOIntRegister    (GPIO_PORTC_BASE, ISRHandlers);                                                                
    GPIOIntRegister		 (GPIO_PORTD_BASE, ISRHandlers);
		
    // interrupt trigger for portA, portC and portD
    GPIOIntTypeSet    (GPIO_PORTA_BASE, Passenger_Elevate_Button		 ,GPIO_RISING_EDGE);
    GPIOIntTypeSet    (GPIO_PORTA_BASE, Driver_Elevate_Button        ,GPIO_RISING_EDGE);
    GPIOIntTypeSet    (GPIO_PORTA_BASE, Passenger_Lower_Button   		 ,GPIO_RISING_EDGE);
    GPIOIntTypeSet    (GPIO_PORTA_BASE, Driver_Lower_Button          ,GPIO_RISING_EDGE);
    GPIOIntTypeSet    (GPIO_PORTC_BASE, Object_Detection_Sensor    	 ,GPIO_BOTH_EDGES);
    GPIOIntTypeSet    (GPIO_PORTC_BASE, Window_Lower_Limit           ,GPIO_BOTH_EDGES);
    GPIOIntTypeSet    (GPIO_PORTC_BASE, Window_Upper_Limit           ,GPIO_BOTH_EDGES);
    GPIOIntTypeSet    (GPIO_PORTC_BASE, Window_Lock_Switch           ,GPIO_BOTH_EDGES);
		GPIOIntTypeSet		(GPIO_PORTD_BASE, ENC_PIN_A | ENC_PIN_B				 ,GPIO_BOTH_EDGES);
		
    //set interrupt priorities for portA, portC and portD
    IntPrioritySet(INT_GPIOC_TM4C123, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    IntPrioritySet(INT_GPIOA_TM4C123, configMAX_SYSCALL_INTERRUPT_PRIORITY+1);
    IntPrioritySet(INT_GPIOD_TM4C123, configMAX_SYSCALL_INTERRUPT_PRIORITY+2);
    
    //enable interrupt for portA, portC and portD
    GPIOIntEnable        (GPIO_PORTA_BASE, GPIO_INT_PIN_2);
    GPIOIntEnable        (GPIO_PORTA_BASE, GPIO_INT_PIN_3);
    GPIOIntEnable        (GPIO_PORTA_BASE, GPIO_INT_PIN_4);
    GPIOIntEnable        (GPIO_PORTA_BASE, GPIO_INT_PIN_5);
    GPIOIntEnable        (GPIO_PORTC_BASE, GPIO_INT_PIN_4);
    GPIOIntEnable        (GPIO_PORTC_BASE, GPIO_INT_PIN_5);
    GPIOIntEnable        (GPIO_PORTC_BASE, GPIO_INT_PIN_6);
    GPIOIntEnable        (GPIO_PORTC_BASE, GPIO_INT_PIN_7);
		GPIOIntEnable				 (GPIO_PORTD_BASE, ENC_PIN_A | ENC_PIN_B);
    
    //creatubg tasks
    xTaskCreate(vPUpTask        , "PassengerWindowElevate"    , 150, NULL, 1, &xPassengerWindowElevateTaskHandle);
    xTaskCreate(vPLowerTask            , "PassengerWindowLower"        , 150, NULL, 1, &xPassengerWindowLowerTaskHandle);
    xTaskCreate(vDUpTask            , "DriverWindowElevate"            , 150, NULL, 2, &xDriverWindowElevateTaskHandle);
    xTaskCreate(vDLowerTask                , "DriverWindowLower"                , 150, NULL, 2, &xDriverWindowLowerTaskHandle);
    xTaskCreate(vLockWindows                                    , "LockWindows"                            , 100, NULL, 2, &xLockWindowsTaskHandle);
    xTaskCreate(vUpperLimit                            , "UpperLimitAction"                , 100, NULL, 4, &xUpperLimitActionHandle);
    xTaskCreate(vLowerLimit                            , "LowerLimitAction"                , 100, NULL, 4, &xLowerLimitActionHandle);
    xTaskCreate(vODetection                        , "ObstacleDetection"                , 100, NULL, 3, &xObsatcleDetectionHandle);
		xTaskCreate(vEncoderTask											, "Encoder", 128, NULL, 1, &xEncoderTaskHandle);
		
    //semaphores
    xDriverWindowElevateTaskUnlockerSemaphore            = xSemaphoreCreateBinary();
    xDriverWindowLowerTaskUnlockerSemaphore                = xSemaphoreCreateBinary();
    xPassengerWindowElevateTaskUnlockerSemaphore    = xSemaphoreCreateBinary();
    xPassengerWindowLowerTaskUnlockerSemaphore        = xSemaphoreCreateBinary();
    xLockWindowsTaskUnlockerSemaphore                        = xSemaphoreCreateBinary();
    xObstacleDetectionUnlockerSemaphore                        = xSemaphoreCreateBinary();
    xLowerLimitUnlockerSemaphore                   = xSemaphoreCreateBinary();
    xUpperLimitUnlockerSemaphore                                    = xSemaphoreCreateBinary();

    LCD_Clear();
    LCD_SetCursor(0, 0);
    LCD_Print(" Welcome to our");
    LCD_SetCursor(1, 0);
    LCD_Print("  Smart System");
    
    vTaskStartScheduler();

    while (1) 
        {
            /* Should not reach here */
        }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//TASKS

void vDUpTask(void *pvParameters) {
    xSemaphoreTake(xDriverWindowElevateTaskUnlockerSemaphore, 0);
    while (1){
        xSemaphoreTake(xDriverWindowElevateTaskUnlockerSemaphore, portMAX_DELAY);
        // delay for debounce
        vTaskDelay(800/portTICK_RATE_MS);
        
        driver_elevate_button_state = GPIOPinRead(Buttons_Motor_Port, Driver_Elevate_Button);
        //manual
        if ( driver_elevate_button_state == HIGH && window_state != WINDOW_CLOSED){
            // Update LCD to show operation
            LCD_Clear();
            LCD_SetCursor(0, 0);
            LCD_Print("Driver Manual");
            LCD_SetCursor(1, 0);
            LCD_Print("Window Moving Up");
            // while its pressed
            while (driver_elevate_button_state == HIGH && window_state != WINDOW_CLOSED){
                // Enable the motor first
                GPIO_PORTF_DATA_R |= DC_Motor_Enable;
                // move window up
                GPIO_PORTA_DATA_R |= DC_Motor_In1;
                GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
                operation = UP;
                window_state = MIDDLE;
                driver_elevate_button_state = GPIOPinRead(Buttons_Motor_Port, Driver_Elevate_Button);
                last_task = DU;
            }
            // Disable the motor first
            GPIO_PORTF_DATA_R &= ~DC_Motor_Enable;
            // Then clear direction pins
            GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
            GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
            last_task = STOP;
            // Update LCD status
            LCD_Clear();
            LCD_SetCursor(0, 0);
            LCD_Print("Window: Middle");
            LCD_SetCursor(1, 0);
            LCD_Print("Manual Up Done");
        }
        //automatic
        else if (driver_elevate_button_state == LOW && window_state != WINDOW_CLOSED){
            // Update LCD to show operation
            LCD_Clear();
            LCD_SetCursor(0, 0);
            LCD_Print("Driver Auto");
            LCD_SetCursor(1, 0);
            LCD_Print("Window Moving Up");
            // Enable the motor first
            GPIO_PORTF_DATA_R |= DC_Motor_Enable;
            // move window up
            GPIO_PORTA_DATA_R |= DC_Motor_In1;
            GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
            operation = UP;
            window_state = MIDDLE;
            last_task = DU;
        }
    }
}

void vDLowerTask(void *pvParameters) {
    xSemaphoreTake(xDriverWindowLowerTaskUnlockerSemaphore, 0);
    while (1){
        xSemaphoreTake(xDriverWindowLowerTaskUnlockerSemaphore, portMAX_DELAY);
        vTaskDelay(800/portTICK_RATE_MS);       
        driver_lower_button_state = GPIOPinRead(Buttons_Motor_Port, Driver_Lower_Button);
        if ( driver_lower_button_state == HIGH && window_state != WINDOW_OPEN){
            LCD_Clear();
            LCD_SetCursor(0, 0);
            LCD_Print("Driver Manual");
            LCD_SetCursor(1, 0);
            LCD_Print("Window Moving Down");
            while (driver_lower_button_state == HIGH && window_state != WINDOW_OPEN){
                // Enable the motor first
                GPIO_PORTF_DATA_R |= DC_Motor_Enable;
                // move window down
                GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
                GPIO_PORTA_DATA_R |= DC_Motor_In2;
                operation = DOWN;
                window_state = MIDDLE;
                last_task = DD;
                driver_lower_button_state = GPIOPinRead(Buttons_Motor_Port, Driver_Lower_Button);
            }
            // Disable the motor first
            GPIO_PORTF_DATA_R &= ~DC_Motor_Enable;
            // Then clear direction pins
            GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
            GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
            last_task = STOP;
            // Update LCD status
            LCD_Clear();
            LCD_SetCursor(0, 0);
            LCD_Print("Window: Middle");
            LCD_SetCursor(1, 0);
            LCD_Print("Manual Down Done");
        }
        else if (driver_lower_button_state == LOW && window_state != WINDOW_OPEN){
            // Update LCD to show operation
            LCD_Clear();
            LCD_SetCursor(0, 0);
            LCD_Print("Driver Auto");
            LCD_SetCursor(1, 0);
            LCD_Print("Window Moving Down");
            // Enable the motor first
            GPIO_PORTF_DATA_R |= DC_Motor_Enable;
            // move window down
            GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
            GPIO_PORTA_DATA_R |= DC_Motor_In2;
            operation = DOWN;
            window_state = MIDDLE;
            last_task = DD;
        }
    }
}

void vPUpTask(void *pvParameters) {
		xSemaphoreTake(xPassengerWindowElevateTaskUnlockerSemaphore, 0);
    while (1){
        xSemaphoreTake(xPassengerWindowElevateTaskUnlockerSemaphore, portMAX_DELAY);
        vTaskDelay(800/portTICK_RATE_MS);
        passenger_elevate_button_state = GPIOPinRead(Buttons_Motor_Port, Passenger_Elevate_Button);
        // MANUAL MODE
        if ( passenger_elevate_button_state == HIGH && window_state != WINDOW_CLOSED){
            // Update LCD to show operation
            LCD_Clear();
            LCD_SetCursor(0, 0);
            LCD_Print("Passenger Manual");
            LCD_SetCursor(1, 0);
            LCD_Print("Window Moving Up");
            
            while (passenger_elevate_button_state == HIGH && window_state != WINDOW_CLOSED){
                // Enable the motor first
                GPIO_PORTF_DATA_R |= DC_Motor_Enable;
                // move window up
                GPIO_PORTA_DATA_R |= DC_Motor_In1;
                GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
                last_task = PU;
                operation = UP;
                window_state = MIDDLE;
                passenger_elevate_button_state = GPIOPinRead(Buttons_Motor_Port, Passenger_Elevate_Button);
            }
            // Disable the motor first
            GPIO_PORTF_DATA_R &= ~DC_Motor_Enable;
            // Then clear direction pins
            GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
            GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
            last_task = STOP;
            // Update LCD status
            LCD_Clear();
            LCD_SetCursor(0, 0);
            LCD_Print("Window: Middle");
            LCD_SetCursor(1, 0);
            LCD_Print("Manual Up Done");
        }
        // AUTOMATIC MODE
        else if (passenger_elevate_button_state == LOW && window_state != WINDOW_CLOSED){
            // Update LCD to show operation
            LCD_Clear();
            LCD_SetCursor(0, 0);
            LCD_Print("Passenger Auto");
            LCD_SetCursor(1, 0);
            LCD_Print("Window Moving Up");
            // Enable the motor first
            GPIO_PORTF_DATA_R |= DC_Motor_Enable;
            // move window up
            GPIO_PORTA_DATA_R |= DC_Motor_In1;
            GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
            operation = UP;
            last_task = PU;
            window_state = MIDDLE;
        }
    }
}

void vPLowerTask(void *pvParameters) {
    xSemaphoreTake(xPassengerWindowLowerTaskUnlockerSemaphore, 0);
    while (1){
        xSemaphoreTake(xPassengerWindowLowerTaskUnlockerSemaphore, portMAX_DELAY);
        vTaskDelay(800/portTICK_RATE_MS);
        passenger_lower_button_state = GPIOPinRead(Buttons_Motor_Port, Passenger_Lower_Button);
        // MANUAL MODE
        if ( passenger_lower_button_state == HIGH && window_state != WINDOW_OPEN){
            // Update LCD to show operation
            LCD_Clear();
            LCD_SetCursor(0, 0);
            LCD_Print("Passenger Manual");
            LCD_SetCursor(1, 0);
            LCD_Print("Window Moving Down");
            while (passenger_lower_button_state == HIGH && window_state != WINDOW_OPEN){
                // Enable the motor first
                GPIO_PORTF_DATA_R |= DC_Motor_Enable;
                // move window down
                GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
                GPIO_PORTA_DATA_R |= DC_Motor_In2;
                operation = DOWN;
                last_task = PD;
                window_state = MIDDLE;
                passenger_lower_button_state = GPIOPinRead(Buttons_Motor_Port, Passenger_Lower_Button);
            }
            // Disable the motor first
            GPIO_PORTF_DATA_R &= ~DC_Motor_Enable;
            // Then clear direction pins
            GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
            GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
            last_task = STOP;
            
            // Update LCD status
            LCD_Clear();
            LCD_SetCursor(0, 0);
            LCD_Print("Window: Middle");
            LCD_SetCursor(1, 0);
            LCD_Print("Manual Down Done");
        }
        else if (passenger_lower_button_state == LOW && window_state != WINDOW_OPEN){
            // Update LCD to show operation
            LCD_Clear();
            LCD_SetCursor(0, 0);
            LCD_Print("Passenger Auto");
            LCD_SetCursor(1, 0);
            LCD_Print("Window Moving Down");
            
            // Enable the motor first
            GPIO_PORTF_DATA_R |= DC_Motor_Enable;
            // move window down
            GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
            GPIO_PORTA_DATA_R |= DC_Motor_In2;
            operation = DOWN;
            last_task = PD;
            window_state = MIDDLE;
        }
    }
}

void vLockWindows(void *pvParameters) {
    xSemaphoreTake(xLockWindowsTaskUnlockerSemaphore, 0);
    while (1){
				// Turn buzzer on
				GPIO_PORTD_DATA_R |= buzzen;
				// Simple delay - consider replacing with a timer-based delay for better accuracy
				for(volatile int i = 0; i < 1000000; i++);
				// Turn buzzer off
				GPIO_PORTD_DATA_R &= ~buzzen;
        xSemaphoreTake(xLockWindowsTaskUnlockerSemaphore, portMAX_DELAY);
        // delay for debounce
        vTaskDelay(500/portTICK_RATE_MS);
        lock_state = GPIOPinRead(Sensors_Port,Window_Lock_Switch);
        if (lock_state == HIGH){
            vTaskSuspend(xPassengerWindowElevateTaskHandle);
            vTaskSuspend(xPassengerWindowLowerTaskHandle);
            
            // Update LCD with lock status
            LCD_Clear();
            LCD_SetCursor(0, 0);
            LCD_Print("Window Control");
            LCD_SetCursor(1, 0);
            LCD_Print("Status: LOCKED");
            
            xSemaphoreTake(xLockWindowsTaskUnlockerSemaphore, portMAX_DELAY);
        }
        else{
            vTaskResume(xPassengerWindowElevateTaskHandle);
            vTaskResume(xPassengerWindowLowerTaskHandle);
            
            // Update LCD with unlock status
            LCD_Clear();
            LCD_SetCursor(0, 0);
            LCD_Print("Window Control");
            LCD_SetCursor(1, 0);
            LCD_Print("Status: UNLOCKED");
            
            xSemaphoreTake(xLockWindowsTaskUnlockerSemaphore, portMAX_DELAY);
        }
    }
}

void vODetection(void *pvParameters) {
    xSemaphoreTake(xObstacleDetectionUnlockerSemaphore, 0);
    while (1) {
				// Turn buzzer on
				GPIO_PORTD_DATA_R |= buzzen;
				// Simple delay - consider replacing with a timer-based delay for better accuracy
				for(volatile int i = 0; i < 1000000; i++);
				// Turn buzzer off
				GPIO_PORTD_DATA_R &= ~buzzen;
        xSemaphoreTake(xObstacleDetectionUnlockerSemaphore, portMAX_DELAY);
        vTaskDelay(200/portTICK_RATE_MS);
        ir_sensor_state = GPIOPinRead(Sensors_Port,Object_Detection_Sensor);
        if ( ir_sensor_state == LOW && operation == UP) { 
            // Update LCD with obstacle detection
            LCD_Clear();
            LCD_SetCursor(0, 0);
            LCD_Print("OBSTACLE DETECTED!");
            LCD_SetCursor(1, 0);
            LCD_Print("Reversing window...");
            
            // Disable the motor first
            GPIO_PORTF_DATA_R &= ~DC_Motor_Enable;
            // Then clear direction pins
            GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
            GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
            vTaskDelay(1000/portTICK_RATE_MS);
            // Enable the motor first
            GPIO_PORTF_DATA_R |= DC_Motor_Enable;
            // Set motor direction to move down
            GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
            GPIO_PORTA_DATA_R |= DC_Motor_In2;
            vTaskDelay(1000/portTICK_RATE_MS);
            // Disable the motor first
            GPIO_PORTF_DATA_R &= ~DC_Motor_Enable;
            // Then clear direction pins
            GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
            GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
            vTaskSuspend(xDriverWindowElevateTaskHandle);
            vTaskSuspend(xPassengerWindowElevateTaskHandle);
            // Update LCD with window status after reversal
            LCD_Clear();
            LCD_SetCursor(0, 0);
            LCD_Print("Safety Activated");
            LCD_SetCursor(1, 0);
            LCD_Print("Window: Middle");
        }
        else {
            vTaskResume(xDriverWindowElevateTaskHandle);
            vTaskResume(xPassengerWindowElevateTaskHandle);
        }
    }
}

void vUpperLimit(void *pvParameters) {
    xSemaphoreTake(xUpperLimitUnlockerSemaphore, 0);
    while (1) {
				// Turn buzzer on
				GPIO_PORTD_DATA_R |= buzzen;
				// Simple delay - consider replacing with a timer-based delay for better accuracy
				for(volatile int i = 0; i < 1000000; i++);
				// Turn buzzer off
				GPIO_PORTD_DATA_R &= ~buzzen;
        xSemaphoreTake(xUpperLimitUnlockerSemaphore, portMAX_DELAY);
        upper_limit_switch_state = GPIOPinRead(Sensors_Port, Window_Upper_Limit);
        // STOPPING MOTOR AT UPPER LIMIT
        if ( (upper_limit_switch_state == LOW) && (operation == UP)) {  // Function to check if an obstacle is detected
            // Disable the motor first
            GPIO_PORTF_DATA_R &= ~DC_Motor_Enable;
            // Then clear direction pins
            GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
            GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
            window_state = WINDOW_CLOSED;
            vTaskSuspend(xDriverWindowElevateTaskHandle);
            vTaskSuspend(xPassengerWindowElevateTaskHandle);
            // Update LCD with window closed status
            LCD_Clear();
            LCD_SetCursor(0, 0);
            LCD_Print("Window Position");
            LCD_SetCursor(1, 0);
            LCD_Print("Status: CLOSED");
        }
        // RE-ALLOW WINDOW CLOSING OPTION
        else {
            vTaskResume(xDriverWindowElevateTaskHandle);
            vTaskResume(xPassengerWindowElevateTaskHandle);
        }
    }
}

void vLowerLimit(void *pvParameters) {
    xSemaphoreTake(xLowerLimitUnlockerSemaphore, 0);
    while (1) {
				// Turn buzzer on
				GPIO_PORTD_DATA_R |= buzzen;
				// Simple delay - consider replacing with a timer-based delay for better accuracy
				for(volatile int i = 0; i < 1000000; i++);
				// Turn buzzer off
				GPIO_PORTD_DATA_R &= ~buzzen;
        xSemaphoreTake(xLowerLimitUnlockerSemaphore, portMAX_DELAY);
        lower_limit_switch_state = GPIOPinRead(Sensors_Port, Window_Lower_Limit);
				//stop motor at lower limit
        if ((lower_limit_switch_state == LOW) && (operation == DOWN)) {
            // Disable the motor first
            GPIO_PORTF_DATA_R &= ~DC_Motor_Enable;
            // Then clear direction pins
            GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
            GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
            window_state = WINDOW_OPEN;
            vTaskSuspend(xDriverWindowLowerTaskHandle);
            vTaskSuspend(xPassengerWindowLowerTaskHandle);
            // Update LCD with window open status
            LCD_Clear();
            LCD_SetCursor(0, 0);
            LCD_Print("Window Position");
            LCD_SetCursor(1, 0);
            LCD_Print("Status: OPEN");
        }
				//allow windoes to be used again
        else {
            vTaskResume(xDriverWindowLowerTaskHandle);
            vTaskResume(xPassengerWindowLowerTaskHandle);
        }
    }
}

void vApplicationIdleHook( void ) {
	for(int i=0;i<10000000;i++);
	LCD_Clear();
  LCD_SetCursor(0, 0);
  LCD_Print("   Sleep mode");
  LCD_SetCursor(1, 0);
  LCD_Print("Team 18 on fire!");       
	lock_state = GPIOPinRead(Sensors_Port,Window_Lock_Switch);
	//disable passenger use
	if (lock_state == HIGH){
		vTaskSuspend(xPassengerWindowElevateTaskHandle);
		vTaskSuspend(xPassengerWindowLowerTaskHandle);
	}
	//low power mode, zzzzzz
  SysCtlSleep();
}

void vEncoderTask(void *pvParameters)
{
    // Initialize display
    LCD_Clear();
    LCD_SetCursor(0,0);
    LCD_Print("Encoder:");
    LCD_SetCursor(1,0);
    {
        char buf[16];
        snprintf(buf, sizeof(buf), "Pos:%2ld Dir:%2ld", encoderPos, encoderDirection);
        LCD_Print(buf);
    }

    for(;;)
    {
        // Wait for notification from ISR
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Clamp range
        if(encoderPos < ENC_MIN_POS) encoderPos = ENC_MIN_POS;
        if(encoderPos > ENC_MAX_POS) encoderPos = ENC_MAX_POS;

        // Display updated position and direction info for debugging
        LCD_SetCursor(1,0);
        {
            char buf[16];
            snprintf(buf, sizeof(buf), "Pos:%2ld Dir:%2ld", encoderPos, encoderDirection);
            LCD_Print(buf);
        }
        
        // Map encoder to window position if needed
        if(encoderPos <= 2) {
            window_state = WINDOW_OPEN;
        } else if(encoderPos >= 8) {
            window_state = WINDOW_CLOSED;
        } else {
            window_state = MIDDLE;
        }
				
				if(encoderPos == 0){
						GPIO_PORTF_DATA_R &= ~DC_Motor_Enable;
						// Then clear direction pins
						GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
						GPIO_PORTA_DATA_R &= ~DC_Motor_In2;

						window_state = WINDOW_OPEN;

						vTaskSuspend(xDriverWindowLowerTaskHandle);
						vTaskSuspend(xPassengerWindowLowerTaskHandle);
						// Update LCD with window open status
						LCD_Clear();
						LCD_SetCursor(0, 0);
						LCD_Print("Window Position");
						LCD_SetCursor(1, 0);
						LCD_Print("Status: OPEN");
						}
				else{
					vTaskResume(xDriverWindowLowerTaskHandle);
          vTaskResume(xPassengerWindowLowerTaskHandle);
				}
				
				
				if(encoderPos == 10){
            // Disable the motor first
            GPIO_PORTF_DATA_R &= ~DC_Motor_Enable;
            // Then clear direction pins
            GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
            GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
            window_state = WINDOW_CLOSED;
            vTaskSuspend(xDriverWindowElevateTaskHandle);
            vTaskSuspend(xPassengerWindowElevateTaskHandle);
            
            // Update LCD with window closed status
            LCD_Clear();
            LCD_SetCursor(0, 0);
            LCD_Print("Window Position");
            LCD_SetCursor(1, 0);
            LCD_Print("Status: CLOSED");
        }
        else {
            vTaskResume(xDriverWindowElevateTaskHandle);
            vTaskResume(xPassengerWindowElevateTaskHandle);
        }
			}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//ISR HANDLER

void ISRHandlers()
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	// WE ALWAYS CLEAR THE INTERRUPT FLAG BEFORE EACH ISR HANDLING
	
	// if interrupt from passenger up
  if (GPIOIntStatus(GPIO_PORTA_BASE, Passenger_Elevate_Button) == Passenger_Elevate_Button)
  {
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_INT_PIN_2);
		lock_state = GPIOPinRead(Sensors_Port,Window_Lock_Switch);
		if (lock_state == HIGH){
			//
		}
		else if(lock_state == LOW && last_task == STOP){
			xSemaphoreGiveFromISR(xPassengerWindowElevateTaskUnlockerSemaphore, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
		else if(lock_state == LOW && (last_task == PU || last_task == DU || last_task == DD || last_task == PD)){
			// Disable the motor first
			GPIO_PORTF_DATA_R &= ~DC_Motor_Enable;
			// Then clear direction pins
			GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
			GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
			last_task = STOP;
		}
  }
	
	
	// int from pass lower
  else if (GPIOIntStatus(GPIO_PORTA_BASE, Passenger_Lower_Button) == Passenger_Lower_Button)
  {	
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_INT_PIN_3);
		lock_state = GPIOPinRead(Sensors_Port,Window_Lock_Switch);
		
		if (lock_state == HIGH){
			//
		}
		else if(lock_state == LOW && last_task == STOP){
			xSemaphoreGiveFromISR(xPassengerWindowLowerTaskUnlockerSemaphore, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
		else if(lock_state == LOW && (last_task == PU || last_task == DU || last_task == DD || last_task == PD)){
			// Disable the motor first
			GPIO_PORTF_DATA_R &= ~DC_Motor_Enable;
			// Then clear direction pins
			GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
			GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
			last_task = STOP;
		}
  }
	
	
  // int from driver up
  else if (GPIOIntStatus(GPIO_PORTA_BASE, Driver_Elevate_Button) == Driver_Elevate_Button)
  {
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_INT_PIN_4);
		if(last_task == STOP){
			xSemaphoreGiveFromISR(xDriverWindowElevateTaskUnlockerSemaphore, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
		else if(last_task == PU || last_task == DU || last_task == DD || last_task == PD){
			// Disable the motor first
			GPIO_PORTF_DATA_R &= ~DC_Motor_Enable;
			// Then clear direction pins
			GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
			GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
			last_task = STOP;
		}
  }
	
	
	// int from driver lower
  else if (GPIOIntStatus(GPIO_PORTA_BASE, Driver_Lower_Button) == Driver_Lower_Button)
  {
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_INT_PIN_5);
		if(last_task == STOP){
			xSemaphoreGiveFromISR(xDriverWindowLowerTaskUnlockerSemaphore, &xHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
		else if(last_task == PU || last_task == DU || last_task == DD || last_task == PD){
			// Disable the motor first
			GPIO_PORTF_DATA_R &= ~DC_Motor_Enable;
			// Then clear direction pins
			GPIO_PORTA_DATA_R &= ~DC_Motor_In1;
			GPIO_PORTA_DATA_R &= ~DC_Motor_In2;
			last_task = STOP;
		}
  }
	
	
	// int from passing object
  else if (GPIOIntStatus(GPIO_PORTC_BASE, Object_Detection_Sensor) == Object_Detection_Sensor)
  {
    GPIOIntClear(GPIO_PORTC_BASE, GPIO_INT_PIN_4);
    xSemaphoreGiveFromISR(xObstacleDetectionUnlockerSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
	
	
	// int from lower limit
  else if (GPIOIntStatus(GPIO_PORTC_BASE, Window_Lower_Limit) == Window_Lower_Limit)
  {
    GPIOIntClear(GPIO_PORTC_BASE, GPIO_INT_PIN_6);
    xSemaphoreGiveFromISR(xLowerLimitUnlockerSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
	
	
	// int from upper limit
  else if (GPIOIntStatus(GPIO_PORTC_BASE, Window_Upper_Limit) == Window_Upper_Limit)
  {
    GPIOIntClear(GPIO_PORTC_BASE, GPIO_INT_PIN_5);
    xSemaphoreGiveFromISR(xUpperLimitUnlockerSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
	
	
	// int from lock switch
  else if (GPIOIntStatus(GPIO_PORTC_BASE, Window_Lock_Switch) == Window_Lock_Switch)
  {
    GPIOIntClear(GPIO_PORTC_BASE, GPIO_INT_PIN_7);
    xSemaphoreGiveFromISR(xLockWindowsTaskUnlockerSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
	
	
	// int from encoder
	else if(GPIOIntStatus(GPIO_PORTD_BASE, ENC_PIN_A | ENC_PIN_B)) 
	{
			BaseType_t woken = pdFALSE;
			uint32_t status = GPIOIntStatus(GPIO_PORTD_BASE, true);
			GPIOIntClear(GPIO_PORTD_BASE, status);
			uint8_t pinA = (GPIO_PORTD_DATA_R & ENC_PIN_A) ? 1 : 0;
			uint8_t pinB = (GPIO_PORTD_DATA_R & ENC_PIN_B) ? 1 : 0;
			uint32_t currentTime = xTaskGetTickCount();
			if ((currentTime - lastEncoderTime) < ENC_DEBOUNCE_MS) {
					return;
			}
			lastEncoderTime = currentTime;
			//check what pin changed
			if (status & ENC_PIN_A) {  			// Pin A changed
					if (pinA == pinB) {    			// If A and B are the same
							encoderPos--;
							encoderDirection = -1;
					} else {
							encoderPos++;
							encoderDirection = 1;
					}
			}
			else if (status & ENC_PIN_B) {  // Pin B changed
					if (pinA != pinB) {    			// If A and B are different
							encoderPos--;
							encoderDirection = -1;
					} else {
							encoderPos++;
							encoderDirection = 1;
					}
			}
			if (encoderPos < ENC_MIN_POS) encoderPos = ENC_MIN_POS;
			if (encoderPos > ENC_MAX_POS) encoderPos = ENC_MAX_POS;
			
			xTaskNotifyFromISR(xEncoderTaskHandle, 0, eNoAction, &woken);
			portYIELD_FROM_ISR(woken);
	}
}