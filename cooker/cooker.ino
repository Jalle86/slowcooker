#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <RotaryEncoder.h>

#include "vars.h"
#include "interpreter.h"

#define TEMP_MAX 95
#define TEMP_MIN 45

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

// WiFi module, deprecated?
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

#define AT_DELAY 200

enum direction
{
  COUNTERCLOCKWISE,
  CLOCKWISE,
};

enum time_unit
{
  HOURS, MINUTES, SECONDS,
};

typedef void (*mode_func)(enum direction);
typedef int fixed; //fixed point number, one decimal precision
typedef void (*print_func)(int, int);

void print_timer(int col, int row);
void print_target(int col, int row);
void print_temperature(int col, int row, float temp);

template<typename T, print_func f>
void display_print(T &v, int col, int row);

void cw_event(void);
void ccw_event(void);
void btn_event(void);

void init_timer(void);
void update_temperature(void);
void pci_setup(byte pin);
void update_display(void);

int rcv_wifi_data(Stream &stream, char *buffer);
void send_wifi_data(Stream &stream, const char *data);
bool send_stream(Stream &from, Stream &to);

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
int target = TEMP_MIN; // desired temperature
const float interval = 0.5; //temperature interval
uint32_t timer = 0; //countdown measured in seconds
fixed temperature;
bool activated = false;

LiquidCrystal lcd(RS, RW, ENABLE, DB4, DB5, DB6, DB7);
RotaryEncoder enc(D1, D2, SWITCH, cw_event, ccw_event, btn_event);
OneWire tempWire(ONEWIRE);
DallasTemperature sensors(&tempWire);
SoftwareSerial BT(BT_RX, BT_TX, false);

template <enum time_unit n>
void print_time(int col, int row, int val)
{
  static int old = 0;
  char num_string[3] = { 0 };
  
  if (val != old)
  {
    old = val;
    sprintf(num_string, "%02d", val);
    
    noInterrupts();
    lcd.setCursor(col, row);
    lcd.print(num_string);
    interrupts();
  }
}

void print_timer(int col, int row)
{
  int hours, minutes;
  uint32_t tmp = timer;

  // hours
  hours = tmp / 3600;
  print_time<HOURS>(col, row, hours);

  // minutes
  tmp %= 3600;
  minutes = tmp / 60;
  print_time<MINUTES>(col + 3, row, minutes);

  // seconds
  tmp %= 60;
  print_time<SECONDS>(col + 6, row, tmp);
}

void print_target(int col, int row)
{
  noInterrupts();
  lcd.setCursor(col, row);
  lcd.print("  "); //erase previous entry
  lcd.setCursor(col, row);
  lcd.print(target);
  interrupts();
}

void print_temperature(int col, int row)
{
  char str[5];
  //format "xx.x" degrees
  sprintf(str, "%02d%c%d", temperature / 10, '.', temperature % 10);
  
  noInterrupts();
  lcd.setCursor(col, row);
  lcd.print(str);
  interrupts();
}

void cw_event(void)
{
  modes[current_mode](CLOCKWISE);
}

void ccw_event(void)
{
  modes[current_mode](COUNTERCLOCKWISE);
}

void btn_event(void)
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
  Serial.println(temperature);

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

template<typename T, print_func f>
void display_print(T &v, int col, int row)
{
  static int old = 0;

  // only update display if the content has changed
  if(v != old)
  {
    f(col, row);
    old = v;
  }
}

void update_display(void)
{  
  static int temp_old = 0;
  static int target_old = 0;
  static int timer_old = 0;

  display_print<int, print_temperature>
    (temperature, TEMP_X, TEMP_Y);
  display_print<int, print_target>
    (target, TARGET_X, TARGET_Y);
  display_print<uint32_t, print_timer>
    (timer, TIMER_X, TIMER_Y);

  noInterrupts();
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
  BT.begin(9600);

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

  send_AT_cmd(Serial, "UART_DEF=9600,8,1,0,0", NULL);
  send_AT_cmd(Serial, "CIPMUX=1", NULL);
  send_AT_cmd(Serial, "CIPSERVER=1,9001", NULL);
  send_wifi_data(Serial, "ping");

  send_stream(Serial, BT);
}

int rcv_wifi_data(Stream &stream, char *buffer)
{
  int i = 0;
  int length = 0;
  char nums[5];

  if (stream.available())
  {
    delay(AT_DELAY);
    if (!stream.find("IPD,0,")) //start of incoming data
      return 0;

    // find length of data,
    while(nums[i] = stream.read(), nums[i++] != ':');

    nums[i] = '\0';
    length = strtol(nums, NULL, 10);

    for (i = 0; i < length; i++)
      buffer[i] = stream.read();

    while(stream.available()) //empty input buffer
      stream.read();
  }

  return length;
}

// send data from one stream to another stream
bool send_stream(Stream &from, Stream &to)
{
  if (!from.available())
    return false;

  while(from.available())
  {
    to.write(from.read());
    delay(1);
  }

  return true;
}

void send_wifi_data(Stream &stream, const char *data)
{
  stream.print("AT+CIPSEND=0,");
  stream.print(strlen(data) + 2);
  stream.print("\r\n");
  delay(AT_DELAY);
  stream.print(data);
  stream.print("\r\n");
  delay(AT_DELAY);
}

void send_AT_cmd(Stream &s, const char *cmd, char *buffer)
{
  int i = 0;
  
  s.print("AT+");
  s.print(cmd);
  s.print("\r\n");
  delay(AT_DELAY);
  
  if (buffer)
    for (i = 0; s.available(); i++)
      buffer[i] = s.read();
  else
    while(s.available())
      s.read();
}

void update_IO(void)
{
  char buffer[30], result[10];

  if (send_stream(BT, Serial))
  {
    delay(AT_DELAY);
    send_stream(Serial, BT);
  }

  if (rcv_wifi_data(Serial, buffer))
  {
    interpret(buffer, result);
    send_wifi_data(Serial, result);
  }
}

void update_tick(void)
{
  static int temperature_timer = 0;

  if (activated)
    --timer > 0 ? timer : 0;

  switch(temperature_timer++)
  {
  case 0:
    // request temperature readings and read on next tick
    sensors.requestTemperatures();
    break;

  case 1:
    update_temperature();
    break;

  case 10:
    temperature_timer = 0;
    break;
  }

  if (!timer)
    activated = false;
}

void loop(void)
{
  update_IO();

  if (tick)
  {
    tick = false;
    update_tick();
  }

  update_display();
  digitalWrite(LED, activated);
}

