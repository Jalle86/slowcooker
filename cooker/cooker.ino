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

enum direction
{
  COUNTERCLOCKWISE,
  CLOCKWISE,
};

typedef void (*mode_func)(enum direction);
typedef int fixed; //fixed point number, one decimal place

void print_timer(int col, int row);
void print_target(int col, int row);
void print_temperature(int col, int row, float temp);

void inc_counter(void);
void dec_counter(void);
void button_down(void);

void init_timer(void);
void update_temperature(void);
void pci_setup(byte pin);
void update_display(void);

void change_target(enum direction d);
void change_status(enum direction d);
void change_hours(enum direction d);
void change_minutes(enum direction d);
void change_seconds(enum direction d);

bool tick = false; //one tick every second

mode_func modes[] =
{
  change_target,
  change_hours,
  change_minutes,
  change_seconds,
  change_status,
};
int current_mode = 0;

// cooker properties
int target = 40; // desired temperature
const float interval = 2;
uint32_t timer = 0; //timer measured in seconds
fixed temperature;
bool activated;

LiquidCrystal lcd(RS, RW, ENABLE, DB4, DB5, DB6, DB7);
RotaryEncoder enc(D1, D2, SWITCH, inc_counter, dec_counter, button_down);
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
  
  uint32_t tmp = timer;
  
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

void inc_counter(void)
{
  modes[current_mode](CLOCKWISE);
}

void dec_counter(void)
{
  modes[current_mode](COUNTERCLOCKWISE);
}

void button_down(void)
{
  current_mode = ++current_mode % (sizeof(modes) / sizeof(*modes));
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
}

SIGNAL(TIMER1_COMPA_vect)
{
  tick = true;
  
  if (activated)
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

void wifi_command(SoftwareSerial &wifi)
{
    char input_buffer[100];
    char result_buffer[10];
    int pos = 0;
    
    wifi.listen();
      
    while (wifi.available())
    {
      input_buffer[pos++] = wifi.read();
      delay(1);
    }
    
    if (!pos) //no command input
      return;

    input_buffer[pos] = '\0';
    
    Serial.println(input_buffer);
    interpret(input_buffer, result_buffer);
    
    wifi.print("AT+CIPSEND=0,");
    wifi.print(strlen(result_buffer));
    wifi.print("\r\n");
    delay(1000);
    wifi.print(result_buffer);
}

void serial_command(SoftwareSerial &my_serial)
{
    char input_buffer[100];
    char result_buffer[10];
    int pos = 0;
    
    my_serial.listen();
      
    while (my_serial.available())
    {
      input_buffer[pos++] = my_serial.read();
      delay(1);
    }
    
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
  static int timer_old = 0;
  
  noInterrupts(); //make sure interrupts don't mess with the lcd
  
  if (temperature != temp_old)
  {
    print_temperature(TEMP_X, TEMP_Y);
    temp_old = temperature;
  }
    
  if (target != target_old)
  {
    print_target(TARGET_X, TARGET_Y);
    target_old = target;
  }
  
  if (timer != timer_old)
  {
    print_timer(TIMER_X, TIMER_Y);
    timer_old = timer;
  }
  
  lcd.setCursor(15, 0);
  lcd.print(current_mode);
  interrupts();
}

void change_target(enum direction d)
{
  if (d == CLOCKWISE)
    target = (target == TEMP_MAX) ? TEMP_MAX : target + 1;
  else
    target = (target == TEMP_MIN) ? TEMP_MIN : target - 1;
}

void change_status(enum direction d)
{
  if (timer)
    activated = !activated;
    
  if (activated) //timer starting, reset timer counter
      TCNT1 = 0;
}

void change_hours(enum direction d)
{
  int hours = timer / 3600;
  
  if (d == CLOCKWISE && hours < 99)
    timer += 3600;
  else if (d == COUNTERCLOCKWISE && hours > 0)
    timer -= 3600;
}

void change_minutes(enum direction d)
{
  int minutes = (timer % 3600) / 60;
  if (d == CLOCKWISE && minutes < 59)
    timer += 60;
  else if (d == COUNTERCLOCKWISE && minutes > 0)
    timer -= 60;
}

void change_seconds(enum direction d)
{
  int seconds = (timer % 3600) % 60;
  if (d == CLOCKWISE && seconds < 59)
    timer++;
  else if (d == COUNTERCLOCKWISE && seconds > 0)
    timer--;
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
  WF.begin(14400); //28800
  
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
  print_temperature(TEMP_X, TEMP_Y);
  
  BT.listen();
  Serial.print("AT+UART_DEF=9600,8,1,0,0\r\n");
  delay(1000);
  Serial.print("AT+CIPMUX=1\r\n");
  delay(1000);
  Serial.print("AT+CIPSERVER=1,9001\r\n");
  delay(1000);
  Serial.print("AT+CIPSEND=0,6\r\n");
  delay(1000);
  Serial.print("ping\r\n");
  delay(1000);
  /*WF.listen();
  delay(500);
  WF.print("AT+CIPMUX=1\r\n");
  delay(1000);
  WF.print("AT+CIPSERVER=1,9001\r\n");
  delay(1000);
  /*WF.print("AT+CIPSEND=0,5\r\n");
  delay(1000);
  WF.println("test");
  WF.print("AT+CIPCLOSE=0\r\n");
  /*delay(1000);
  /*WF.print("+IPD,0,5\r\n");
  delay(1000);
   while(WF.available())
     Serial.write(WF.read());
   Serial.println("row");*/
  
  while(Serial.available())
    BT.write(Serial.read());
}

void loop(void)
{
    static int temperature_timer = 0;
    
    //serial_command(BT);
    //wifi_command(WF);
    
    /*while(WF.available())
    {
      digitalWrite(LED, 1);
      Serial.write(WF.read());
    }*/
    
    while(BT.available())
    {
      Serial.write(BT.read());
      delay(1);
    }
    
    while(Serial.available())
    {
      delay(500);
      char buf[20];
      Serial.find("IPD,0,");
      int i;
      int num = Serial.read() - 48;
      Serial.read();
      for (i = 0; i < num; i++)
        buf[i] = Serial.read();
      buf[num] = 0;
      BT.println(buf);
      while(Serial.available())
        Serial.read();
    }
    
    if (tick)
    {      
      tick = false;
      if (activated)
        --timer > 0 ? timer : 0;
          
      temperature_timer++;
      
      // get temperature readings and read on next tick
      if (temperature_timer == 9)
      {
        sensors.requestTemperatures();
      }
      else if (temperature_timer == 10)
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
