# Lib Documentation
## usart communication
init
```c++
#include "lib/usart.h"
#define F_CPU 16000000
#define FOSC 16000000 // Clock Speed of arduino nano 
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1

USART_init(MYUBRR);
```
usage
```c++
USART_println (char *s)

USART_print (char *s)
```
## adc - analog digital converter
init
```c++
ADC_Init();
```
usage
```c++
//read channel 0 20 times and average
adcval = ADC_Read_Avg(0,20);
//read channel 4 one time
adcval= = ADC_Read( 4 );
```