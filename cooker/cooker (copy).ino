#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <RotaryEncoder.h>
#include <EEPROM.h>

#include "vars.h"
#include "interpreter.h"

//#define WIFI_ENABLED
#define DEBUG_ENABLED

#define TEMP_DEFAULT 50
#define INTERVAL     5
#define TEMP_PERIOD  20
#define AUTH_TIMEOUT 600

/* * * * * * * * * * *
 * Pin designations  *
 * * * * * * * * * * */

// lcd display
#define DB4      5
#define DB5      4
#define DB6      3
#define DB7      2
#define RS       8
#define RW       7
#define ENABLE   6

// rotary encoder
#define D1      11
#define D2      12
#define SWITCH  13

// temperature sensor
#define ONEWIRE 9

// BT module
#define BT_TX   18 //A4, goes to rx on module through voltage divider
#define BT_RX   19 //A5, goes to tx on module

// WiFi module [deprecated?]
//#define WIFI_TX 14
//#define WIFI_RX 15

#define PWR_SWITCH 14

#define RELAY   10

#define LED     17

// coordinates on LCD display
#define TIMER_X 1
#define TIMER_Y 0
#define TEMP_X 1
#define TEMP_Y 1
#define TARGET_X 10
#define TARGET_Y 0
#define LCD_W  16
#define LCD_H  2

#define AT_DELAY 200

#define CLOCK byte(0)
#define SENSOR byte(1)
#define ARROW byte (2)

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
typedef void (*switch_func)(bool b);
typedef void (*print_func)(int, int);
typedef int (*rcv_func)(Stream &, char *);
typedef void (*send_func)(Stream &, const char *);
typedef int32_t fixed; //fixed point number, one decimal precision

int freeRam(void);

bool interpret_stream_auth(Stream &stream);
bool interpret_stream(Stream &stream, rcv_func rcv, send_func snd);

int rcv_wifi_data_ip(Stream &stream, char *buffer, char ip[4]);
int rcv_wifi_data(Stream &stream, char *buffer);
void send_wifi_data(Stream &stream, const char *data);

int rcv_bt_data(Stream &stream, char *buffer);
void send_bt_data(Stream &stream, const char *data);

bool stream_pipeline(Stream &from, Stream &to);
void send_AT_cmd(Stream &s, const char *cmd, char *buffer);
bool get_authentication(Stream &stream);

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
void change_hours(enum direction d);
void change_minutes(enum direction d);
void change_seconds(enum direction d);

bool tick = false; //one tick every second

// Custom 8x5 LCD characters
byte clock[8] =
{
  B00000,
  B00100,
  B01110,
  B01110,
  B01110,
  B11111,
  B00100,
  B00000,
};

byte tmp_sprite[8] =
{
  B00100,
  B01010,
  B01010,
  B01110,
  B01110,
  B11111,
  B11111,
  B01110,
};

byte arrow[8] =
{
  B10000,
  B11000,
  B11100,
  B11110
  B11100,
  B11000,
  B10000,
  B00000,
};

struct lcd_option =
{
  int *val;
  char *menu_text;
  char *change_text;
  mode_func func;
};

struct button_funcs =
{
  mode_func rot;
  switch_func sw;
};

lcd_option options[] =
{
  { &target, "Target", "Target temp.:", change_target },
  { &timer, "Timer", "Timer:", change_time },
};

struct button_funcs mode[] =
{
};

int menu_position = 0;
mode_func current = move_cursor();


void display_view(void)
{
  lcd.setCursor(TIMER_X - 1, TIMER_Y);
  lcd.write(CLOCK);
  
  lcd.setCursor(TIMER_X, TIMER_Y);
  lcd.print("00:00:00");

  lcd.setCursor(TEMP_X - 1, TEMP_Y);
  lcd.write(SENSOR);
  
  lcd.setCursor(TEMP_X + 4, TEMP_Y);
  lcd.write(0xDF); //degree sign
  lcd.write('C');
  
  display_print<print_temperature>(temperature, TEMP_X, TEMP_Y);
  display_print<print_timer>(timer, TIMER_X, TIMER_Y);
}

// dummy function
void move_view(enum direction d)
{
}

void view_transition(bool b)
{
  current = 
}

#define SET_MENU_POS(X) (lcd.setCursor(1 + 7 % (X), (X) / 2))
void display_menu(void)
{
  int i;
  menu_position = 0;
  for (i = 0; i < sizeof(options); i++)
  {
    SET_MENU_POS(i);
    lcd.print(option[i].menu_text);
    col = !col;
  }
  
  SET_MENU_POS(i);
  lcd.print("Back");
  
  display_cursor();
}

void display_cursor(void)
{
  SET_MENU_POS(menu_position);
  lcd.write(ARROW);
}

void move_cursor(enum direction d)
{
  SET_MENU_POS(menu_position);
  lcd.write(0x01); //clear cursor
  
  if (d == COUNTERCLOCKWISE)
    menu_position--;
  else //CLOCKWISE
    menu_position++;
    
  menu_position %= sizeof(options) + 1; //add one for back option
  
  display_cursor();
}

void menu_transition(bool b)
{
  if menu_position < sizeof(options)
  {
  }
}

// LCD Related
mode_func modes[] =
{
  change_target,
  change_hours,
  change_minutes,
  change_seconds,
};
int current_mode;


// IP address of WIFI-module, represented as a string for ease of use with interpreter 
char esp_ip[16] = { "0.0.0.0" };

bool authenticated;
const char *auth_phrase = "1234567890";
int auth_timer;
char auth_ip[4];

// cooker properties
int32_t target = TEMP_DEFAULT; // desired temperature
const fixed interval = INTERVAL; //temperature interval
int32_t timer; //countdown measured in seconds
fixed temperature;
int32_t activated;
char ssid[SSID_LEN];
char password[PW_LEN];

LiquidCrystal lcd(RS, RW, ENABLE, DB4, DB5, DB6, DB7);
RotaryEncoder enc(D1, D2, SWITCH, cw_event, ccw_event, btn_event);
OneWire tempWire(ONEWIRE);
DallasTemperature sensors(&tempWire);
SoftwareSerial BT(BT_RX, BT_TX, false);

int freeRam(void) 
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
    
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

  #ifdef DEBUG_ENABLED
  Serial.println(temperature);
  auth_timer = 0;
  #endif // DEBUG_ENABLED
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
}

void change_target(enum direction d)
{
  if (d == CLOCKWISE)
    target = (target == TEMP_MAX) ? TEMP_MAX : target + 1;
  else
    target = (target == TEMP_MIN) ? TEMP_MIN : target - 1;
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
void update_array_eeprom(int addr)
{
  int i;
  static bool written = false;
  static char old_buffer[length];
  
  if (!written)
  {
    read_array_eeprom(old_buffer, length, addr);
    written = true;
  }
  
  for (i = 0; i < length; i++)
    // only update new values to eeprom, to avoid wear and tear
    if (buffer[i] != old_buffer[i])
    {
      EEPROM.write(addr + i, buffer[i]);
      old_buffer[i] = buffer[i];
    }
}

void setup(void)
{
  char buffer[100];
  buffer[0] = 0;
  char *strpos;
  
  sensors.begin();
  enc.begin();
  Serial.begin(9600);
  lcd.begin(LCD_W, LCD_H);
  pci_setup(BT_RX);
  pci_setup(BT_TX);
  BT.begin(9600);

  pinMode(RELAY, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(PWR_SWITCH, INPUT_PULLUP);
  digitalWrite(RELAY, LOW);
  digitalWrite(LED, LOW);

  print_target(TARGET_X, TARGET_Y);

  lcd.createChar(0, clock);
  lcd.setCursor(TIMER_X - 1, TIMER_Y);
  lcd.write(byte(0));
  
  lcd.setCursor(TIMER_X, TIMER_Y);
  lcd.print("00:00:00");

  lcd.createChar(1, tmp_sprite);
  lcd.setCursor(TEMP_X - 1, TEMP_Y);
  lcd.write(byte(1));
  
  lcd.setCursor(TEMP_X + 4, TEMP_Y);
  lcd.write(0xDF); //degree sign
  lcd.write('C');

  lcd.createChar(2, arrow);
  
  init_timer();

  //do not block while reading data from sensor
  sensors.setWaitForConversion(false);

  #ifdef WIFI_ENABLED
  //allow multiple connections
  send_AT_cmd(Serial, "+CIPMUX=1", NULL);
  //set server timeout to 600 seconds
  send_AT_cmd(Serial, "+CIPSTO=600", NULL);
  // show ip on received messages
  send_AT_cmd(Serial, "+CIPDINFO=1", NULL);
  // create server
  send_AT_cmd(Serial, "+CIPSERVER=1,9001", NULL);
  
  // get local ip address
  send_AT_cmd(Serial, "+CIFSR", buffer);
  if (strpos = strstr(buffer, "STAIP,"))
  {
    int i = 0;
    strpos+=7; //go to beginning of ip address
    for (i = 0; *(strpos + i) != '"'; i++)
      esp_ip[i] = *(strpos + i);
    esp_ip[i] = '\0';
  }

  read_array_eeprom(ssid, SSID_LEN, 0);
  read_array_eeprom(password, PW_LEN, SSID_LEN);
  
  #endif // WIFI_ENABLED
  
  // connect to AP
  //sprintf(buffer, "+CWJAP=\"%s\",\"%s\"", ssid, password);
  //send_AT_cmd(Serial, buffer, NULL);
  //send_wifi_data(Serial, "ping");
}
bool interpret_stream_auth(Stream &stream)
{
  char buffer[50], buffer_out[20];
  char ip[4];
  bool same_ip;
  int i;
  
  if (authenticated && rcv_wifi_data_ip(stream, buffer, ip))
  {
    same_ip = true;
    for (i = 0; i < 4; i++)
      same_ip &= (ip[i] == auth_ip[i]);
      
    if (same_ip)
    {
      interpret(buffer, buffer_out);
      auth_timer = 0;
    }
    else
      sprintf(buffer_out, "%s", "Not authenticated");
      
    send_wifi_data(stream, buffer_out);
  }
  else if (!authenticated)
    authenticated = get_authentication(stream);
}

bool interpret_stream(Stream &stream, rcv_func rcv, send_func snd)
{
  char in_buffer[30], out_buffer[30];
  if (rcv(stream, in_buffer))
  {    
    interpret(in_buffer, out_buffer);
    update_array_eeprom<ssid, SSID_LEN>(0);
    update_array_eeprom<password, PW_LEN>(SSID_LEN);
    snd(stream, out_buffer);
    
    return true;
  }
  else
    return false;
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
  rcv_wifi_data_ip(stream, buffer, NULL);
}

// Receive data from ESP-01 module.
int rcv_wifi_data_ip(Stream &stream, char *buffer, char ip[4])
{
  int i = 0, j;
  int length = 0;
  char nums[5];

  if (stream.available())
  {
    delay(AT_DELAY);
    if (!stream.find("IPD,0,")) //start of incoming data
      return 0;

    // find length of data,
    // assuming AT+CIPDINFO=1
    do
      nums[i] = stream.read();
      while (nums[i++] != ',');

    nums[i] = '\0'; //append end of string
    length = strtol(nums, NULL, 10);

    if (ip)
      for(j = 0, i = 0; j < 4; i++)
      {
        nums[i] = stream.read();
        if (nums[i] == '.' || nums[i] == ',') //end of IP byte
        {
          nums[i] = '\0';
          ip[j++] = strtol(nums, NULL, 10);
          i = 0;
        }
      }

    stream.find(":"); //beginning of data
    for (i = 0; i < length; i++)
      buffer[i] = stream.read();

    while(stream.available()) //empty input buffer
      stream.read();
      
    buffer[length] = 0;
    BT.println(buffer);
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

// pipeline the output of one stream to the input of the other.
bool stream_pipeline(Stream &from, Stream &to)
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
  
  s.print("AT");
  s.print(cmd);
  s.print("\r\n");
  delay(AT_DELAY);
  
  // return answer to AT command
  if (buffer)
  {
    for (i = 0; s.available(); i++)
      buffer[i] = s.read();
    buffer[i] = '\0';
  }
  else // no output buffer provided, empty stream buffer
    while(s.available())
      s.read();
}

// read data from ESP-01 module, and return whether the correct
// authorization phrase has been entered.
bool get_authentication(Stream &stream)
{
  int i;
  char buffer[50];
  char ip[4];
  
  if (rcv_wifi_data_ip(stream, buffer, ip))
  {
    if (!strcmp(buffer, auth_phrase))
    {
      send_wifi_data(stream, "Authenticated");
      for(i = 0; i < 4; i++)
        auth_ip[i] = ip[i];
      
      auth_timer = 0;
      return true;
    }
    else
    {
      send_wifi_data(stream, "Authentication failed");
      return false;
    }
  }
}

void update_IO(void)
{
  #ifdef WIFI_ENABLED
  interpret_stream_auth(Serial);
  #endif //WIFI_ENABLED
  
  /*if (stream_pipeline(BT, Serial))
  {
    delay(AT_DELAY);
    stream_pipeline(Serial, BT);
  }*/
    
  interpret_stream(BT, rcv_bt_data, send_bt_data);
}

void update_tick(void)
{
  static int temperature_timer = 0;

  if (activated)
    --timer > 0 ? timer : 0;

  #ifdef WIFI_ENABLED
  if (authenticated && ++auth_timer == AUTH_TIMEOUT)
  {
    authenticated = false;
    auth_timer = 0;
    send_wifi_data(Serial, "Authentication timed out.");
  }
  #endif //WIFI_ENABLED
  switch(temperature_timer++)
  {
  case 0:
    // request temperature readings and read on next tick
    sensors.requestTemperatures();
    break;

  case 1:
    update_temperature();
    break;

  case TEMP_PERIOD:
    temperature_timer = 0;
    break;
  }

  if (!timer)
    activated = false;
}

void loop(void)
{  
  static bool foo = false;
  update_IO();
  
  if (!digitalRead(PWR_SWITCH) && !foo)
  {
    activated = !activated;
    foo = true;
    
    if (activated) //timer starting, reset timer counter
      TCNT1 = 0;
  }
  else if (digitalRead(PWR_SWITCH) && foo)
    foo = false;

  if (tick)
  {
    update_tick();
    tick = false;
  }

  update_display();
  digitalWrite(LED, activated);
  
  if (!activated)
    digitalWrite(RELAY, LOW);
  else if (temperature < 10 * target - interval)
    digitalWrite(RELAY, HIGH);
  else if (temperature > 10 * target + interval)
    digitalWrite(RELAY, LOW);
}

