#include <ADC_mega16.hpp>

char    dec_ASCII[5] = "";     // char buffers
char    voltage_buf[12] = "";
char    temp_buf[10] = "";
uint8_t channel_position = 0;   // channel selector
uint8_t ADC_channels[8] = {
  0b00000, 0b00001, 0b00010, 0b00011,
  0b00100, 0b00101, 0b00110, 0b00111
};

void init_ADC(){
  // Prescaler = 8
  // Enable Auto trigger mode
  ADCSRA |= (1 << ADPS0) | (1 << ADPS1) | (1 << ADATE);

  // reference voltage -> AVCC, pada minsys, AREF tidak ada header pinout-nya
  // ADMUX = VREF_INT_2_56;
  ADMUX = VREF_AVCC;
  ADMUX &= ~(1 << ADLAR);

  // Auto trigger source = Free running
  SFIOR &= ~((1 << ADTS2) | (1 << ADTS1) | (1 << ADTS0));

  // Enable ADC & start the first conversion
  ADCSRA |= (1 << ADEN) | (1 << ADSC);
}

uint16_t baca_ADC(int channel){
  uint16_t ADC_bufferL;
  uint8_t ADC_bufferH;
  uint8_t channel_mask = channel;

  ADMUX &= 0xE0;
  ADMUX |= channel_mask;

  // clear interrupt flag
  ADCSRA |= (1 << ADIF);

  // tunggu ADIF bernilai 1
  while((ADCSRA & (1 << ADIF)) == 0);

  // ambil low nibble terlebih dahulu, agar nilai ADCH & ADCL
  // di lock selama proses fungsi baca_ADC() ini
  ADC_bufferL = ADCL;
  ADC_bufferH = ADCH;
  ADC_bufferL |= (ADC_bufferH << 8);

  // ambil high nibble (ADCH) yang sudah digeser ke kiri 8x, lalu di OR
  // dengan hasil low nibble (ADCL)
  // ADC_buffer |= (ADCH << 8);

  // clear interrupt flag
  ADCSRA |= (1 << ADIF);

  return ADC_bufferL;
}


// HELPER FUNCTIONS ---------------------------------
void dec2ASCII(uint16_t ADC_value){
  sprintf(dec_ASCII, "%d%d%d%d",
    ADC_value / 1000,
    (ADC_value / 100) % 10,
    (ADC_value / 10) % 10,
    ADC_value % 10
  );
}

double dec2volt(uint16_t ADC_value, uint8_t channel_type)
{
  double LSB;
  if (channel_type == SINGLE_ENDED){
    LSB = 5.0 / 1024.0;
  }
  else if (channel_type == DIFFERENTIAL_INPUT){
    LSB = 5.0 / 512.0;
  }
  return ADC_value * LSB;
}

void volt2ASCII(double voltage_value)
{
  double voltage_mV = voltage_value * 1000;
  unsigned int voltage_integer = voltage_mV;
  unsigned int voltage_comma_values = (voltage_mV - voltage_integer) * 100;

  sprintf(voltage_buf, "%d%d%d%d.%d%dmV", 
           voltage_integer / 1000,
          (voltage_integer / 100) % 10,
          (voltage_integer / 10) % 10,
           voltage_integer % 10,
          (voltage_comma_values / 10) % 10,
           voltage_comma_values % 10);
}
