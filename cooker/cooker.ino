#include <LiquidCrystal.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <RotaryEncoder.h>

#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>

#define DB4      5
#define DB5      4
#define DB6      3      
#define DB7      2

#define RS       6
#define RW       7
#define ENABLE   8

#define D1      11
#define D2      12
#define SWITCH  13

#define ONEWIRE 9

#define RELAY   10

#define TEMP_MAX 80
#define TEMP_MIN 40

void print_target(void);
void print_temp(void);
void inc_temp(void);
void dec_temp(void);
void foo(void);

int target = 50;
const float interval = 0.1;

LiquidCrystal lcd(RS, RW, ENABLE, DB4, DB5, DB6, DB7);
RotaryEncoder enc(D1, D2, SWITCH, inc_temp, dec_temp, foo);
OneWire tempWire(ONEWIRE);
DallasTemperature sensors(&tempWire);

void print_target(void)
{
  lcd.home();
  lcd.write(' ');
  lcd.write(' ');
  lcd.home();
  
  lcd.print(target);
}

void print_temperature(float temp)
{
  static int oldtemp = 0;
  char str[10];
  lcd.setCursor(0, 1);
  
  //sadly enough sprintf is not supported for floats on the Arduino
  //platform. We have to convert the value to an int and work from
  //there
  int tempint = temp * 10;
  //format "xx.x" degrees
  sprintf(str, "%02d%c%d", tempint / 10, '.', tempint % 10);
  if (tempint != oldtemp)
  {
    lcd.print(str);
    oldtemp = tempint;
  }
}

void inc_temp(void)
{
  int old = target;
  target = target == TEMP_MAX ? TEMP_MAX : target + 1;
  
  if (target != old)
    print_target();
}

void dec_temp(void)
{
  int old = target;
  target = target == TEMP_MIN ? TEMP_MIN : target - 1;
  
  if (target != old)
    print_target();
}

void foo(void)
{
}

void init_timer(void)
{
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  
  //set timer every 4 seconds
  OCR1A = 62500;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12); //1/1024 prescaler
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}

ISR (PCINT0_vect)
{
  enc.update();
}

SIGNAL(TIMER1_COMPA_vect)
{
  sensors.requestTemperatures();
  delay(1);
  float t = sensors.getTempCByIndex(0);
  
  if (t < target - interval)
    digitalWrite(RELAY, HIGH);
  else if (t > target + interval)
    digitalWrite(RELAY, LOW);
    
  print_temperature(t);
}

void setup(void)
{
  sensors.begin();
  enc.begin();
  Serial.begin(9600);
  lcd.begin(16, 2);
  
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, LOW);
      
  print_target();
  
  lcd.setCursor(4, 1);
  lcd.write(0xDF); //degree sign
  lcd.write('C');
  
  init_timer();
}

void loop(void)
{
    // Choose our preferred sleep mode:
    set_sleep_mode(SLEEP_MODE_IDLE);
 
    // Set sleep enable (SE) bit:
    sleep_enable();
 
    // Put the device to sleep:
    sleep_mode();
 
    // Upon waking up, sketch continues from this point.
    sleep_disable();
}
