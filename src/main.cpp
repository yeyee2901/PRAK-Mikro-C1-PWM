#include <Arduino.h>
#include <driver_LCD16x2.hpp>
#include <ADC_mega16.hpp>
#include <stdio.h>

// GLOBAL MACROS -------------------------------------------
#ifndef TRUE
  #define TRUE  1
  #define FALSE 0
#endif

#define CW_DIR  TRUE
#define CCW_DIR FALSE


// GLOBAL VARIABLES ----------------------------------------
// flags
bool              motor_running = FALSE;
bool              motor_dir = CW_DIR;
volatile uint8_t  motor_change_dir = 0;
volatile uint8_t  start_stop_btn = 0;


// FUNCTION PROTOTYPING ------------------------------------
// for 8-bit Fast PWM mode only!!
void     pwm_init(void);
void     pwm_write(uint8_t);
void     start_motor(uint8_t);
void     stop_motor(uint8_t);


// MAIN FUNCTION -------------------------------------------
void setup() {

  // Init LCD
  lcd_init(&PORTC, &PORTD, &DDRC, &DDRD, PD0, PD1);
  lcd_command(CLEAR_DISPLAY);

  // Init ADC (channel 1 saja)
  DDRA  &= ~(1 << PD1);
  PORTA  = 0x00;
  init_ADC();

  // init PWM & motor pin
  pwm_init();
  DDRB |= (1 << PB0) | (1 << PB1);
  PORTB = 0;

  // EXTERNAL INTERRUPT - untuk kontrol motor
  // INT0 falling edge trigger - start/stop
  // INT1 falling edge trigger - direction
  GICR  |= (1 << INT0) | (1 << INT1);
  MCUCR |= (1 << ISC01) | (1 << ISC11);

  // global interrupt enable
  sei();
  delay(2000);
}

void loop() {

  lcd_command(CLEAR_DISPLAY);

  // procces pergantian arah
  if(motor_change_dir > 0)
  {
    lcd_setpos(0,0);
    lcd_string(">changing");
    lcd_setpos(1,0);
    lcd_string("direction...");

    delay(3000);
    if(motor_dir == CW_DIR)
    {
      motor_dir = CCW_DIR;
      PORTB = (1 << PB0);
    }
    else
    {
      motor_dir = CW_DIR;
      PORTB = (1 << PB1);
    }

    // clear flag counter
    motor_change_dir = 0;
    lcd_command(CLEAR_DISPLAY);
  }

  uint16_t ADC_value = baca_ADC(1); // channel 1

  // mapping 10-bit value ke 8-bit value
  uint8_t PWM_val = ADC_value / 4;
  lcd_setpos(0,0);
  lcd_char((unsigned char) ((PWM_val / 100)%10 + 0x30));
  lcd_char((unsigned char) ((PWM_val / 10 )%10 + 0x30));
  lcd_char((unsigned char) ((PWM_val % 10 )    + 0x30));

  // get duty cycle
  uint8_t duty = 100 * PWM_val / 255;
  lcd_setpos(0,9);
  lcd_char((unsigned char) ((duty / 100)%10 + 0x30));
  lcd_char((unsigned char) ((duty / 10 )%10 + 0x30));
  lcd_char((unsigned char) ((duty % 10 )    + 0x30));
  lcd_char('%');

  // jika button ditekan & motor sedang tidak running
  // maka nyalakan motor
  if((start_stop_btn > 0) && (motor_running == FALSE))
  {
    lcd_setpos(1,0);
    lcd_string("Starting motor!");
    start_motor(PWM_val);
    delay(2000);

    // update flags
    start_stop_btn = 0;
    motor_running = TRUE;
  }

  // jika button ditekan & motor sedang running
  // maka matikan motor
  if((start_stop_btn > 0) && (motor_running == TRUE))
  {
    lcd_setpos(1,0);
    lcd_string("Stopping motor!");
    stop_motor(PWM_val);
    delay(2000);

    // update flags
    start_stop_btn = 0;
    motor_running = FALSE;
  }
  
  lcd_setpos(1,0);
  lcd_string("               ");

  // jika motor sedang running, maka bisa atur kecepatan
  if(motor_running)
  {
    pwm_write(PWM_val);
    lcd_setpos(1,0);
    lcd_string("Direction: ");

    if(motor_dir == CCW_DIR)
    {
      PORTB = (1 << PB1);
      lcd_string("CCW");
    }
    else
    {
      PORTB = (1 << PB0);
      lcd_string("CW");
    }
  }

  // jika tidak, maka tampilkan informasi saja
  else{
    lcd_setpos(1,0);
    lcd_string("Motor is OFFzz");
  }

  delay(500);

}


// ISR ----------------------------------------------
// start-stop motor
ISR(INT0_vect)
{
  start_stop_btn++;
}

ISR(INT1_vect)
{
  motor_change_dir++;
}

// FUNCTION DEFINITION ------------------------------
void pwm_init()
{
  DDRD |= (1 << PD5);
  PORTD = 0;
  TCCR1A = 0;
  TCCR1B = 0;

  // Fast PWM mode
  // inverting mode, prescaler 64
  TCCR1A |= (1 << COM1A1) | (1 << WGM10);
  TCCR1B |= (1 << CS11)   | (1 << CS10) | (1 << WGM12);

}


void pwm_write(uint8_t decimal_value)
{
  OCR1A = decimal_value;
}


void start_motor(uint8_t start_value)
{
  PORTB = 0;

  pwm_write(0);
  if(motor_dir == CW_DIR)
  {
    PORTB = (1 << PB0);
  }
  else{
    PORTB = (1 << PB1);
  }

  // tambahkan pwm secara perlahan dari 0
  int temp = 0;
  while(temp < start_value)
  {
    temp += 13;
    if(temp > start_value) temp = start_value;

    pwm_write(temp);
    delay(100);
  }

  if(temp > start_value) 
  {
    temp = start_value;
    pwm_write(start_value);
  }

}


void stop_motor(uint8_t current_value)
{
  pwm_write(current_value);

  // kurangi PWM secara perlahan
  int temp = current_value;
  while(temp > 0)
  {
    temp -= 13;
    if(temp <= 0) temp = 0;

    pwm_write(temp);
    delay(100);

  }
  PORTB = 0;
}
