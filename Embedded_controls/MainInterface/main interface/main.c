#define F_CPU 16000000 // Clock speed
#define FOSC 16000000  // Oscilator frequency
#define BAUD 9600      // BAUD rate
#include "lib/usart.h"
#include "main_interface.h"
#include "schnittstelle.h"
//#define MYUBRR FOSC/16/BAUD-1

int main(void)
{

    initSchnittstelle();

    while (true)
    {

        waitForSerialCommandAndExecuteAndSendStatus();
        _delay_ms(1000);
    }

    return 0;

    /*USART_init(MYUBRR);
    char pvar[5];

    sprintf(pvar, "S");
    USART_println(pvar);
    
    main_init();
    sprintf(pvar, "I");
    USART_println(pvar);
    _delay_ms(3000);
   
    //float center_degs[18] = {90.0, 140.0, 120.0, 90.0, 140.0, 130.0, 90.0, 140.0, 140.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0};
    float center_degs[18] = {90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0, 90.0};

    sprintf(pvar, "%i", (int)status.distance);
    USART_println(pvar);
    sprintf(pvar, "%i", (int)status.gyro_data.rotationX);
    USART_println(pvar);
    sprintf(pvar, "%i", (int)status.gyro_data.rotationY);
    USART_println(pvar);
    sprintf(pvar, "%i", (int)status.gyro_data.rotationZ);
    USART_println(pvar);
    sprintf(pvar, "%i", (int)status.gyro_data.temprature);
    USART_println(pvar);
    _delay_ms(3000);

    while(true){

    }

    return 0;

    */
}
