#include "ultrasonic.h"
#define usInvalidDistance 2185 // Distance returned if a timeout occurs
#define usAcousticVelocity 932.94 // speed factor regarding the atmegas clock oscillator frequnecy of 16MHz (1 Instruction = 0.0625 us, 1 Instruction = 0,00214 cm, Sonic-Speed = 34300 cm/s)

// Init setup status variable
uint8_t setupFinished = 0; 
// Init Timer Overflow variable
volatile uint32_t timerOverflow = 0;
// Do something on interrput
ISR(TIMER0_OVF_vect){
	//Increment Timer Overflow count
	timerOverflow +=1;
}

// setup USART and configure pins
void setup(){
	// Set Trigger Pin as output
	Trigger_DDR |= (1 << Trigger_pin);
	// Set Echo Pin as input
	Echo_DDR &= ~(1 << Echo_pin);
	// Enable pull up at Echo Port (sets configured pin to 1 instead of an undefined, third state)
	Echo_PORT |= (1 << Echo_pin);
	// Use Timer TC0 (8bit timer, which makes sense using a 8-bit controller)
	// Enable Timer/Counter Module: Set PRTIM0 bit to zero in PRR Register to enable Timer/Counter module 0
	PRR &= ~(1 << PRTIM0);
	// Set clock source, prescaler 1. This starts counting.
	TCCR0B = 0b00000001;
	// Enable timer interrupts on overflow for timer 0
	TIMSK0 |= (1 << TOIE0);
	// enable global interrupt
	sei();
    // Remember that this setup finished
    setupFinished = 1;
}

// TODO: Should we disable the counter & interrupts after calling the 'usGetDistanceFloat' function?
void disableUSTimerCounter(){
    // Set clock source to 0. This stops counting.
	TCCR0B = 0b00000001;
	// Disable timer interrupts on overflow for timer 1
	TIMSK0 &= ~(1 << TOIE0);
}

// Trigger the sensor
void trigger(){
	// Set trigger pin high
	Trigger_PORT |= (1 << Trigger_pin);
	// Wait for 10 us
	_delay_us(10);
	// Set trigger pin low again
	Trigger_PORT &= ~(1 << Trigger_pin);
}

// How should it work?
// 1. Set a high signal to trigger pin for 10 microseconds.
// 2. After ~250 microseconds, sensor sets echo pin to high (which means it sent a ultrasonic signal).
// 3. After the ultrasonic signal returned (depending on the distance the sonic signal has to travel), the echo pin is set to low.
// 4. Measuring the time between high and low of the echo pin results in the time the ultrasonic signal travelled.
// 5. Multiply this time with a factor (speed of ultrasonic) in order to calculate the distance.
// Arduino nano doesn't have a 'clock', instead use a 8bit/16bit counter.
double usGetDistanceFloat(){
    // only call setup once
    if(setupFinished == 0){
        setup();
    }
    // Trigger the sensor
    trigger();

    // Wait for rising edge
    while((Echo_PIN & (1 << Echo_pin)) == 0);
    // Reset counter
    TCNT0 = 0;
    // Reset timer overflow
    timerOverflow = 0;
    // Wait for falling edge
    while(!(Echo_PIN & (1 << Echo_pin)) == 0);

    // Get current counter value
    uint32_t currentCounter = TCNT0;
    // Get current overflow value
    uint32_t currentTimerOverflow = timerOverflow;
    // Sum up counter values
    uint32_t counterSum = currentCounter + (256 * currentTimerOverflow);
    // calculate distance as float
    double distance = counterSum / usAcousticVelocity;

    return distance;
}

// Trigger the sensor n times and return the average of all distances
double usGetAverageDistanceFloat(uint8_t numberOfDistances){
    uint8_t numberOfValidDistances = 0;
    double sumOfDistances = 0;
    double averageDistance = 0;
    
    // Get 'numberOfDistances' distances. Only use valid distances
    for(uint8_t idx=0; idx<numberOfDistances; idx++) {
        double distance = usGetDistanceFloat();
		if(distance < usInvalidDistance){
            numberOfValidDistances += 1;
            sumOfDistances += distance;
        }
        // Wait 20ms before the next trigger
        _delay_ms(20);
	}
    
    // Calculate average distance. If there were only invalid distances, return invalid distance
    if(sumOfDistances > 0){
        averageDistance = sumOfDistances / numberOfValidDistances;
    }else{
        averageDistance = usInvalidDistance;
    }

    return averageDistance;
}

