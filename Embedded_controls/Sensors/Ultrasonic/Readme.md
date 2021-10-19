# Ultrasonic Documentation

## Basic principle of work
Ultrasonic ranging module HC - SR04 provides 2cm - 400cm non-contact measurement function, the ranging accuracy can reach to 3mm. 
The modules includes ultrasonic transmitters, receiver and control circuit. 
The basic principle of work: 
1. Set a high signal to trigger pin for 10 microseconds.
1. After ~250 microseconds, sensor sets echo pin to high (which means it sent a ultrasonic signal).
1. After the ultrasonic signal returned (depending on the distance the sonic has to travel), the echo pin is set to low.
1. Measuring the time between high and low of the echo pin results in the time the ultrasonic signal travelled.
1. Multiply this time with a factor (speed of ultrasonic) in order to calculate the distance.

## Interface
### Parameters
Set these parameters in the `ultrasonic.h` file to match your device.
* `Trigger_pin`: Trigger pin bit
* `Echo_pin`: Echo pin bit
* `Echo_PIN`: Echo pin register
* `Trigger_DDR`: Trigger data direction register
* `Echo_DDR`: Echo data direction register
* `Trigger_PORT`: Trigger port
* `Echo_PORT`: Echo port
* `F_CPU`: Clock speed
* `FOSC`: Oscilator frequency
* `BAUD`: BAUD rate

### Functions
* `usGetDistanceFloat()`: Get the current distance to the next object as **double**. If the sensor won't receive the sonic signal (e.g. distance is too far/close), the distance **2185** will be returned. It is recommended to wait 20ms for the next call of this method.
* `usGetAverageDistanceFloat(uint8_t numberOfDistances)`: Get the current distance to the next object as **double**. Triggers the sensor `numberOfDistances` times and averages all valid results. If the sensor won't receive the sonic signal (e.g. distance is too far/close), the distance **2185** will be returned. It is recommended to wait 20ms for the next call of this method.

### Example Usage
```c++
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
```

## Dependencies
* avr/io.h
* avr/interrupt.h
* util/delay.h
* stdio.h
* stdlib.h
* stdint.h
