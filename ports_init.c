#include "ports_init.h"

void PortA_Config(void) {
  //clock  
	SYSCTL_RCGCGPIO_R |= 0x01;
  /*
		pa2, pa3, pa4, pa5 --> inputs 
		p6, p7 				 		 --> outputs	
	*/
	GPIO_PORTA_DIR_R |= 0xC0;
  GPIO_PORTA_DIR_R &= ~(0x3C);
	
  // digital enable for them all
  GPIO_PORTA_DEN_R |= 0xFC;
}


void PortC_Config(void) {
    SYSCTL_RCGCGPIO_R |= 0x04;
    
	  //pc4, pc5, pc6, pc7 --> inputs
    GPIO_PORTC_DIR_R &= ~(0xF0);
    GPIO_PORTC_DEN_R |= 0xF0;
}

void PortF_Config(void) {
    SYSCTL_RCGCGPIO_R |= 0x20;
    while((SYSCTL_RCGCGPIO_R  & 0x20) == 0){}
    
    //unlock port f
    GPIO_PORTF_LOCK_R = 0x4C4F434B;
    GPIO_PORTF_CR_R = 0x1F;
    
    /*
			pf0,pf4 --> inputs,
			pf1, pf2, pf3 --> outputs
		*/
		GPIO_PORTF_DIR_R = 0x0E;
    GPIO_PORTF_DEN_R |= 0x1F;
    
    //initialize motor enable as off
    GPIO_PORTF_DATA_R &= ~(0x02);
}

void PortD_Config(void) {
    SYSCTL_RCGCGPIO_R |= 0x08;
    while((SYSCTL_RCGCGPIO_R & 0x08) == 0){}
    
    //pd6, pd7 as inputs from the encoder
    GPIO_PORTD_DIR_R &= ~(0xC0);  
		
		//pd0 outut for buzzer
    GPIO_PORTD_DIR_R |= 0x01;     
    
    // Enable pull-up resistors for pins 6 and 7
    GPIO_PORTD_PUR_R |= 0xC0;
    
    // Enable digital functionality for pins 0, 6, and 7
    GPIO_PORTD_DEN_R |= 0xC1;  // 0xC1 = 0b11000001 for pins 0, 6, and 7
    
    // Initialize buzzer as off
    GPIO_PORTD_DATA_R &= ~(0x01);  // Clear bit 0 (PD0) for buzzer
}