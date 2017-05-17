#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <RotaryEncoder.h>
#include <EEPROM.h>

#include "vars.h"
#include "interpreter.h"

#define TEMP_MAX     95
#define TEMP_MIN     45
#define TEMP_DEFAULT 83
#define INTERVAL     0.5

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
typedef void (*print_func)(int, int);
typedef int (*rcv_func)(Stream &, char *);
typedef void (*send_func)(Stream &, const char *);
typedef int32_t fixed; //fixed point number, one decimal precision

void interpret_stream(Stream &stream, rcv_func rcv, send_func snd);
int rcv_wifi_data(Stream &stream, char *buffer);
int rcv_bt_data(Stream &stream, char *buffer);
void send_wifi_data(Stream &stream, const char *data);
void send_bt_data(Stream &stream, const char *data);
bool send_stream(Stream &from, Stream &to);
void send_AT_cmd(Stream &s, const char *cmd, char *buffer);

template<print_func f>
void display_print(int32_t v, int col, int row);
template <enum time_unit n>
void print_time(int col, int row, int val);
void print_timer(int col, int row);
void print_target(int col, int row);
void print_temperature(int col, int row, float temp);

void cw_event(void);
void ccw_event(void);
void btn_event(void);

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
int current_mode;

// cooker properties
int32_t target = TEMP_DEFAULT; // desired temperature
const float interval = INTERVAL; //temperature interval
int32_t timer; //countdown measured in seconds
fixed temperature;
int32_t activated = false;
char ssid[20];
char password[20];
 
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
    
    lcd.setCursor(col, row);
    lcd.print(num_string);
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
  lcd.setCursor(col, row);
  lcd.print("  "); //erase previous entry
  lcd.setCursor(col, row);
  lcd.print(target);
}

void print_temperature(int col, int row)
{
  char str[5];
  //format "xx.x" degrees
  int t = temperature; //sprintf doesn't take kindly to int32_t
  sprintf(str, "%02d.%d", t / 10, t % 10);
  
  lcd.setCursor(col, row);
  lcd.print(str);
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
  float t = sensors.getTempCByIndex(0);
  temperature = t * 10; //times 10 because we have 1 decimal precision

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

template<print_func f>
void display_print(int32_t v, int col, int row)
{
  static int32_t old = 0;

  // only update display if the content has changed
  if(v != old)
  {
    f(col, row);
    old = v;
  }
}

void update_display(void)
{  
  display_print<print_temperature>(temperature, TEMP_X, TEMP_Y);
  display_print<print_target>(target, TARGET_X, TARGET_Y);
  display_print<print_timer>(timer, TIMER_X, TIMER_Y);

  lcd.setCursor(15, 0);
  lcd.print(current_mode);
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

void read_array_eeprom(char *buffer, int length, int addr)
{
  int i;
  for (i = 0; i < length; i++)
    buffer[i] = EEPROM.read(addr + i);
}

template<char *buffer, int length>
void write_array_eeprom(int addr)
{
  int i;
  static bool written = false;
  static char old_buffer[length];
  
  for (i = 0; i < length; i++)
    // only update new values if we've written before
    if (!written || buffer[i] != old_buffer[i])
    {
      EEPROM.write(addr + i, buffer[i]);
      old_buffer[i] = buffer[i];
    }
    
   written = true;
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

  //send_AT_cmd(Serial, "UART_DEF=9600,8,1,0,0", NULL);
  //send_AT_cmd(Serial, "CIPMUX=1", NULL);
  //send_AT_cmd(Serial, "CIPSERVER=1,9001", NULL);
  //send_wifi_data(Serial, "ping");

  char buffer[50];
  read_array_eeprom(ssid, SSID_LEN, 0);
  read_array_eeprom(password, PW_LEN, SSID_LEN);
  sprintf(buffer, "CWJAP=\"%s\",\"%s\"", ssid, password);
  send_AT_cmd(Serial, buffer, NULL);
  //send_stream(Serial, BT);
}

void interpret_stream(Stream &stream, rcv_func rcv, send_func snd)
{
  char in_buffer[30], out_buffer[30];
  if (rcv(stream, in_buffer))
  {
    interpret(in_buffer, out_buffer);
    write_array_eeprom<ssid, SSID_LEN>(0);
    write_array_eeprom<password, PW_LEN>(SSID_LEN);
    snd(stream, out_buffer);
  }
}

int rcv_bt_data(Stream &stream, char *buffer)
{
  int length = 0;
  
  while (stream.available())
  {
    buffer[length++] = stream.read();
    delay(1);
  }
  
  buffer[length] = '\0';
  
  return length;
}

void send_bt_data(Stream &stream, const char *data)
{
  int i;
  
  for (i = 0; data[i]; i++)
    stream.write(data[i]);
  
  stream.print("\r\n");
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
  /*if (send_stream(BT, Serial))
  {
    delay(AT_DELAY);
    //send_stream(Serial, BT);
  }
  else if (send_stream(Serial, BT))
  {
    delay(AT_DELAY);
    send_stream(BT, Serial);
  }*/
  
  //interpret_stream(Serial, rcv_wifi_data, send_wifi_data);
  
  interpret_stream(BT, rcv_bt_data, send_bt_data);
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

