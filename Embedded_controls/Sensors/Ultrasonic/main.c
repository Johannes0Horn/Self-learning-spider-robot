#include "ultrasonic.h"
#include "lib/usart.h"
#define MYUBRR FOSC/16/BAUD-1

int main(void){
    USART_init(MYUBRR);
    char pvar[50];
    while(1){
        double distance = usGetAverageDistanceFloat(4);
        sprintf(pvar, "Value of distance: %ld cm", (uint32_t)distance);
        USART_println(pvar);
        _delay_ms(2000);
    }
}