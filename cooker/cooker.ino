#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <RotaryEncoder.h>
#include <EEPROM.h>

#include "vars.h"
#include "display.h"
#include "interpreter.h"

//#define WIFI_ENABLED
#define DEBUG_ENABLED

#define TEMP_DEFAULT 50
#define TEMP_PERIOD  15
#define AUTH_TIMEOUT 600

#define AT_DELAY 200

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

// bluetooth
#define BT_TX   18 //A4, goes to rx on module through voltage divider
#define BT_RX   19 //A5, goes to tx on module

#define PWR_SWITCH 14

#define RELAY   10

#define LED     17

/*  Necessary function prototypes. Arduino IDE prepends function
    prototypes during compilation, but they are declared before
    dependencies such as structs or enums are declared. */
    
typedef void (*print_func)(int, int);
typedef int (*rcv_func)(Stream &, char *);
typedef void (*send_func)(Stream &, const char *);
typedef int32_t fixed; //fixed point number, one decimal precision

bool interpret_stream_auth(Stream &stream);
bool interpret_stream(Stream &stream, rcv_func rcv, send_func snd);

// IP address of WIFI-module, represented as a string for ease of use with interpreter 
char esp_ip[16] = { "0.0.0.0" };

bool authenticated;
const char *auth_phrase = "1234567890";
int auth_timer;
char auth_ip[4];

// cooker properties
int32_t target = TEMP_DEFAULT; // desired temperature
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

bool tick = false; //one tick every second
bool tock = false; //one tock every 10ms
int cycle_length = 10; //10 time units, 1 time unit == 10ms
float duty_cycle;
int kp = 15; //proportional gain
int ki = 0;
int kd = 0;

void update_controller(void)
{
  static fixed error_old = 0;
  
  fixed error = 10 * target - temperature;
  
  fixed integral = (error_old + error) / 2; //trapezoidal rule
  fixed derivative = error - error_old;
  duty_cycle = (kp * error + ki * integral + kd * derivative) / 100;
  
  if (duty_cycle > 1)
    duty_cycle = 1;
  else if (duty_cycle < 0)
    duty_cycle = 0;
    
  error_old = error;
}

void update_relay(void)
{
  static int cycle_time = 0;
  if (cycle_time++ < duty_cycle * 10)
    digitalWrite(RELAY, HIGH);
  else
    digitalWrite(RELAY,LOW);
    
  if (cycle_time >= cycle_length)
    cycle_time = 0;
}

void cw_event(void)
{
  current_mode.change(CLOCKWISE);
}

void ccw_event(void)
{
  current_mode.change(COUNTERCLOCKWISE);
}

void btn_event(void)
{
  lcd.clear();
  current_mode = current_mode.next();
  current_mode.init();
}

void init_timer(void)
{
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  //timer every 10ms
  OCR1A = 625;
  TCCR1B |= (1 << WGM12); //ctc mode

  //1/256 prescaler
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
  
  update_controller();
}

SIGNAL(TIMER1_COMPA_vect)
{
  static int sec_counter = 0;
  
  if (++sec_counter == 100) //one second has passed
  {
    tick = true;
    sec_counter = 0;
  }
  
  if (activated)
    tock = true;
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

  lcd.createChar(0, clock);
  lcd.createChar(1, tmp_sprite);
  lcd.createChar(2, arrow);
  
  init_timer();

  current_mode.init();
  
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

void update_serial(void)
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
  // software solution to avoid contact bounce
  static bool foo = false;
  update_serial();
  
  if (!digitalRead(PWR_SWITCH) && !foo)
  {
    activated = !activated;
    foo = true;
    #ifdef DEBUG_ENABLED
    Serial.println("push");
    #endif
    if (activated) //timer starting, reset timer counter
      TCNT1 = 0;
  }

  if (tick)
  {
    update_tick();
    tick = false;
  }

  if (tock)
  {
    update_relay();
    tock = false;
    foo = false;
  }
  
  current_mode.update();
  
  digitalWrite(LED, activated);
  
  if (!activated)
    digitalWrite(RELAY, LOW);
  // multiply by 10 because fixed point number
  //else if (temperature < 10 * target - interval)
  //  digitalWrite(RELAY, HIGH);
  //else if (temperature > 10 * target + interval)
  //  digitalWrite(RELAY, LOW);*/
}

