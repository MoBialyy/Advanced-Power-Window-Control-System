#ifndef PORT_INIT_H
#define PORT_INIT_H


#include "TM4C123GH6PM.h"
#define SYSCTL_RCGCGPIO_R       (*((volatile unsigned long *)0x400FE608))

//PORTA
#define GPIO_PORTA_DATA_R       (*((volatile uint32_t *)0x400043FC))
#define GPIO_PORTA_DIR_R        (*((volatile uint32_t *)0x40004400))
#define GPIO_PORTA_DEN_R        (*((volatile uint32_t *)0x4000451C))
	
//PORTB
#define GPIO_PORTB_DIR_R        (*((volatile uint32_t *)0x40005400))
#define GPIO_PORTB_DEN_R        (*((volatile uint32_t *)0x4000551C))
	
//PORTC
#define GPIO_PORTC_DIR_R        (*((volatile uint32_t *)0x40006400))
#define GPIO_PORTC_DEN_R        (*((volatile uint32_t *)0x4000651C))
	
//PORTF
#define GPIO_PORTF_DATA_R       (*((volatile unsigned long *)0x400253FC))
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_LOCK_R       (*((volatile unsigned long *)0x40025520))
#define GPIO_PORTF_CR_R         (*((volatile unsigned long *)0x40025524))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))


//PORTD
#define GPIO_PORTD_DATA_R       (*((volatile unsigned long *)0x400073FC))
#define GPIO_PORTD_DIR_R        (*((volatile unsigned long *)0x40007400))
#define GPIO_PORTD_PUR_R        (*((volatile unsigned long *)0x40007510))
#define GPIO_PORTD_DEN_R        (*((volatile unsigned long *)0x4000751C))


#define INT_GPIOA_TM4C123       16          // GPIO Port A
#define INT_GPIOB_TM4C123       17          // GPIO Port B
#define INT_GPIOC_TM4C123       18          // GPIO Port C
#define INT_GPIOD_TM4C123       19          // GPIO Port D
#define INT_GPIOE_TM4C123       20          // GPIO Port E

//gloval definitions
#define Buttons_Motor_Port						GPIO_PORTA_BASE
#define Sensors_Port									GPIO_PORTC_BASE
#define Object_Detection_Sensor				(1<<4)
#define Window_Upper_Limit						(1<<5)
#define Window_Lower_Limit						(1<<6)
#define Window_Lock_Switch						(1<<7)
#define Passenger_Elevate_Button			(1<<2)
#define Passenger_Lower_Button			 	(1<<3)
#define Driver_Elevate_Button					(1<<4)
#define Driver_Lower_Button					 	(1<<5)

#define GPIO_PORTA_BASE         			0x40004000  // GPIO Port A
#define GPIO_PORTB_BASE         			0x40005000  // GPIO Port B
#define GPIO_PORTC_BASE         			0x40006000  // GPIO Port C
#define GPIO_PORTD_BASE         			0x40007000  // GPIO Port D

#define DC_Motor_In1									(1<<6)
#define DC_Motor_In2									(1<<7)
#define DC_Motor_Enable 							 0x02  

#define PC0														(1<<0)
#define PC1														(1<<1)
#define PC2														(1<<2)
#define PC3														(1<<3)
#define PA0														(1<<0)
#define PA1														(1<<1)

#define LOW															0
#define HIGH														1

void PortA_Config(void);
void PortC_Config(void);
void PortF_Config(void);
void PortD_Config(void);

#endif