#ifndef ADC_MEGA16
#define ADC_MEGA16

#include <Arduino.h>


#define DIFFERENTIAL_INPUT  0
#define SINGLE_ENDED        1
#define VREF_AVCC           (1<<REFS0)
#define VREF_INTERNAL       0

extern char    dec_ASCII[5];     // char buffers
extern char    voltage_buf[12];
extern char    temp_buf[10];
extern uint8_t channel_position;   // channel selector
extern uint8_t ADC_channels[8];


// FUNCTION PROTOTYPE --------------------------------------
void     init_ADC();
uint16_t baca_ADC(int channel);
double   dec2volt(uint16_t ADC_value, uint8_t channel_type);
void     volt2ASCII(double voltage_value);
void     dec2ASCII(uint16_t ADC_value); 

#endif