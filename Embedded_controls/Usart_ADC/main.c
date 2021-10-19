// shell Befehl zum Port auszulesen: ls /dev/*
// shell Befehl um Seriellen Port zu empfangen: screen /dev/tty.usbserial-1420 9600

#define F_CPU 16000000
#define FOSC 16000000 // Clock Speed vom arduino nano ist nicht die selbe wie vom atmel chip
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include "lib/usart.h"
#include "lib/adc.h"


int main(){
	USART_init(MYUBRR);
	uint16_t adcval;
	ADC_Init();

	int i=0;
	while(i<10000){
	
		adcval = ADC_Read_Avg(0,20);

    	char puffer[20];
		sprintf( puffer, "- %i",adcval);
	  	USART_println(puffer) ;

    	_delay_ms(1000);
		i=i+1;

	}
}

