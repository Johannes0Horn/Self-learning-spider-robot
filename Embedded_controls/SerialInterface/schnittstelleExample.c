#include "motorcontrol.h"
#include "main_interface.h"


int main(void){
    initSchnittstelle();
    while (true){
        waitForSerialCommandAndExecuteAndSendStatus();
        _delay_ms(1000);
    }
    return 0;
}


