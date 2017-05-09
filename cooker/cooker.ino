#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <RotaryEncoder.h>

#include "vars.h"
#include "interpreter.h"

#define TEMP_MAX 80
#define TEMP_MIN 10

/* * * * * * * * * * *
 * Pin designations  *
 * * * * * * * * * * */
 
// lcd display
#define DB4      5
#define DB5      4
#define DB6      3
#define DB7      2
#define RS       6
#define RW       7
#define ENABLE   8

// rotary encoder
#define D1      11
#define D2      12
#define SWITCH  13

// temperature sensor
#define ONEWIRE 9

// BT module
#define BT_TX   18 //A4, goes to rx on module through voltage divider
#define BT_RX   19 //A5, goes to tx on module

// WiFi module
#define WIFI_TX 14
#define WIFI_RX 15

#define RELAY   10

#define LED     17

// coordinates on LCD display
#define TIMER_X 8
#define TIMER_Y 1
#define TEMP_X 0
#define TEMP_Y 1
#define TARGET_X 0
#define TARGET_Y 0

typedef int fixed; //fixed point number, one decimal place

void print_timer(int col, int row);
void print_target(int col, int row);
void print_temperature(int col, int row, float temp);

void inc_temp(void);
void dec_temp(void);
void button_down(void);

void init_timer(void);
void update_temperature(void);
void pci_setup(byte pin);
void update_display(void);

bool tick = false; //one tick every second

// cooker properties
int target = 40; // desired temperature
const float interval = 2;
int timer = 3600; //timer measured in seconds
fixed temperature;
bool activated;

LiquidCrystal lcd(RS, RW, ENABLE, DB4, DB5, DB6, DB7);
RotaryEncoder enc(D1, D2, SWITCH, inc_temp, dec_temp, button_down);
OneWire tempWire(ONEWIRE);
DallasTemperature sensors(&tempWire);
SoftwareSerial BT(BT_RX, BT_TX, false);
SoftwareSerial WF(WIFI_RX, WIFI_TX, false);

void print_timer(int col, int row)
{
  int hours, minutes;
  static int hours_old = 0;
  static int minutes_old = 0;
  static int seconds_old = 0;
  char num_string[3] = { 0 };
  
  int tmp = timer;
  
  // hours
  hours = tmp / 3600;
  if (hours != hours_old)
  {
    hours_old = hours;
    sprintf(num_string, "%02d", hours);
    lcd.setCursor(col, row);
    lcd.print(num_string);
  }
  
  // minutes
  tmp %= 3600;
  minutes = tmp / 60;
  if (minutes != minutes_old)
  {
    minutes_old = minutes;
    sprintf(num_string, "%02d", minutes);
    lcd.setCursor(col + 3, row);
    lcd.print(num_string);
  }
  
  // seconds
  tmp %= 60;
  if (tmp != seconds_old)
  {
    seconds_old = tmp;
    sprintf(num_string, "%02d", tmp);
    lcd.setCursor(col + 6, row);
    lcd.print(num_string);
  }
}

void print_target(int col, int row)
{
  lcd.setCursor(col, row);
  lcd.print("  "); //erase previous entry
  lcd.setCursor(col, row);
  lcd.print(target);
}

void print_temperature(int col, int row)
{
  char str[5];
  lcd.setCursor(col, row);
  
  //format "xx.x" degrees
  sprintf(str, "%02d%c%d", temperature / 10, '.', temperature % 10);
  
  lcd.print(str);
}

void inc_temp(void)
{
  target = (target == TEMP_MAX) ? TEMP_MAX : target + 1;
}

void dec_temp(void)
{
  target = (target == TEMP_MIN) ? TEMP_MIN : target - 1;
}

void button_down(void)
{
  if (timer)
    activated = !activated;
    
  if (activated) //timer starting, reset timer counter
      TCNT1 = 0;
}

void init_timer(void)
{
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  
  //set timer every second
  OCR1A = 15625;
  TCCR1B |= (1 << WGM12); //ctc mode
  
  //1/1024 prescaler
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  
  TIMSK1 |= (1 << OCIE1A); //timer cmp interrupt
  interrupts();
}

void update_temperature(void)
{
  volatile float t = sensors.getTempCByIndex(0);
  temperature = t * 10;
  
  if (!activated)
    digitalWrite(RELAY, LOW);
  else if (t < target - interval)
    digitalWrite(RELAY, HIGH);
  else if (t > target + interval)
    digitalWrite(RELAY, LOW);
  
  // Let the sensor do it's work while we wait for the next reading
  sensors.requestTemperatures();
}

SIGNAL(TIMER1_COMPA_vect)
{
  tick = true;
  
  print_timer(TIMER_X, TIMER_Y);
}

ISR(PCINT0_vect)
{
  enc.update();
}

// activate pin change interrupt
void pci_setup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

void setup(void)
{
  sensors.begin();
  enc.begin();
  Serial.begin(9600);
  lcd.begin(16, 2);
  pci_setup(BT_RX);
  pci_setup(BT_TX);
  pci_setup(WIFI_RX);
  pci_setup(WIFI_TX);
  BT.begin(9600);
  WF.begin(115200);
  BT.listen();
  
  pinMode(RELAY, OUTPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(RELAY, LOW);
  digitalWrite(LED, LOW);
      
  print_target(TARGET_X, TARGET_Y);
  
  lcd.setCursor(8, 1);
  lcd.print("00:00:00");
  
  lcd.setCursor(4, 1);
  lcd.write(0xDF); //degree sign
  lcd.write('C');
  
  init_timer();
  
  //do not block while reading data from sensor
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures(); //initial reading
  delay(750); //wait for first conversion
  update_temperature();
}

void serial_command(SoftwareSerial &my_serial)
{
    char input_buffer[100];
    char result_buffer[10];
    int pos = 0;
    
    if (my_serial.available())
      delay(5); //make sure we don't read the buffer too fast
      
    while (my_serial.available())
      input_buffer[pos++] = my_serial.read();
    
    if (!pos) //no command input
      return;

    input_buffer[pos] = '\0';
    
    Serial.println(input_buffer);
    interpret(input_buffer, result_buffer);
    my_serial.println(result_buffer);
}

void update_display(void)
{  
  static int temp_old = 0;
  static int target_old = 0;
  
  noInterrupts(); //make sure interrupts don't mess with the lcd
  
  if (temperature != temp_old)
  {
    print_temperature(TEMP_X, TEMP_Y);
    temp_old = temperature;
  }
    
  if (target != target_old)
  {
    print_target(0, 0);
    target_old = target;
  }
  
  interrupts();
}

void loop(void)
{
    static int temperature_timer = 0;
    serial_command(BT);
    
    if (tick)
    {
      tick = false;
      if (activated)
        --timer > 0 ? timer : 0;
          
      temperature_timer++;
      if (temperature_timer == 10)
      {
        update_temperature();
        temperature_timer = 0;
      }
        
      if (!timer)
        activated = false;
    }
    
    update_display();
    digitalWrite(LED, activated);
}
