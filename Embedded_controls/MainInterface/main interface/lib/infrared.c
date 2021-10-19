#include "infrared.h"

void infraredInit(){
    ADC_Init();
}

int infraredGetDirection(){
    uint8_t num_samples = 10;

    uint16_t intensity_front = ADC_Read_Avg(Ir_front_pin, num_samples);
    uint16_t intensity_rear = ADC_Read_Avg(Ir_rear_pin, num_samples);
    uint16_t intensity_right = ADC_Read_Avg(Ir_right_pin, num_samples);
    uint16_t intensity_left = ADC_Read_Avg(Ir_left_pin, num_samples);

    int direction = 0;

    if(intensity_front > intensity_rear && intensity_front > intensity_right && intensity_front > intensity_left){
        direction = 0;
    }else if(intensity_right > intensity_front && intensity_right > intensity_rear && intensity_right > intensity_left){
        direction = 90;
    }else if(intensity_rear > intensity_front && intensity_rear > intensity_right && intensity_rear > intensity_left){
        direction = 180;
    }else if(intensity_left > intensity_front && intensity_left > intensity_right && intensity_left > intensity_rear){
        direction = 270;
    }

    return direction;

}
