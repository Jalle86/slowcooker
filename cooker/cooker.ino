#include <SoftwareSerial.h>
#include <LiquidCrystal.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <RotaryEncoder.h>

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
#define BT_TX 18 //A4, goes to rx on module through voltage divider
#define BT_RX 19 //A5, goes to tx on module

#define RELAY   10

#define LED 17

void print_target(int col, int row);
void print_temperature(int col, int row, float temp);
void print_time(int col, int row);

void inc_temp(void);
void dec_temp(void);
void foo(void);

int target = 21; // desired temperature
const float interval = 2;
int time; //time of day measured in seconds
int timeleft = 10; //timer
int temperature;
bool activated;

LiquidCrystal lcd(RS, RW, ENABLE, DB4, DB5, DB6, DB7);
RotaryEncoder enc(D1, D2, SWITCH, inc_temp, dec_temp, foo);
OneWire tempWire(ONEWIRE);
DallasTemperature sensors(&tempWire);
SoftwareSerial BT(BT_RX, BT_TX, false);

int order_of_magnitude(int num)
{
  int pos = 0;
  while (num /= 10, num > 0)
    pos++;
  
  return pos;
}

void print_time(int col, int row)
{
  int tmp = timeleft;
  
  lcd.setCursor(col, row);
  lcd.print("00:00:00");
  
  // hours
  lcd.setCursor(col + 1 - order_of_magnitude(tmp / 3600), row);
  lcd.print(tmp / 3600);
  
  // minutes
  tmp %= 3600;
  lcd.setCursor(col + 4 - order_of_magnitude(tmp / 60), row);
  lcd.print(tmp / 60);
  
  // seconds
  tmp %= 60;
  lcd.setCursor(col + 7 - order_of_magnitude(tmp), row);
  lcd.print(tmp);
}

void print_target(int col, int row)
{
  lcd.setCursor(col, row);
  lcd.print("  ");
  lcd.setCursor(col, row);
  lcd.print(target);
}

void print_temperature(int col, int row, float temp)
{
  static int oldtemp = 0;
  static int wait = 5;
  char str[10];
  lcd.setCursor(col, row);
  
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
  
  if (!(--wait))
  {
    wait = 5;
    //Serial.println(temp);
  }
}

void inc_temp(void)
{
  int old = target;
  target = target == TEMP_MAX ? TEMP_MAX : target + 1;
  
  if (target != old)
    print_target(0, 0);
}

void dec_temp(void)
{
  int old = target;
  target = target == TEMP_MIN ? TEMP_MIN : target - 1;
  
  if (target != old)
    print_target(0, 0);
}

void foo(void)
{
  activated = !activated;
  digitalWrite(LED, activated);
  Serial.println("ping");
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
  sensors.requestTemperatures();
  delay(1);
  volatile float t = sensors.getTempCByIndex(0);
  
  if (!activated)
    digitalWrite(RELAY, LOW);
  else if (t < target - interval)
    digitalWrite(RELAY, HIGH);
  else if (t > target + interval)
    digitalWrite(RELAY, LOW);
    
  print_temperature(0, 1, t);
}

ISR (PCINT0_vect)
{
  enc.update();
}

SIGNAL(TIMER1_COMPA_vect)
{
  update_temperature();
  if (activated)
    timeleft = timeleft > 0 ? timeleft - 1 : 0;
  print_time(8, 1);
}

void pciSetup(byte pin)
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
  pciSetup(BT_RX);
  pciSetup(BT_TX);
  BT.begin(9600);
  
  pinMode(RELAY, OUTPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(RELAY, LOW);
  digitalWrite(LED, LOW);
      
  print_target(0, 0);
  
  lcd.setCursor(4, 1);
  lcd.write(0xDF); //degree sign
  lcd.write('C');
  
  init_timer();
}

void loop(void)
{
    // Choose our preferred sleep mode:
    /*set_sleep_mode(SLEEP_MODE_IDLE);
 
    // Set sleep enable (SE) bit:
    sleep_enable();
 
    // Put the device to sleep:
    sleep_mode();
 
    // Upon waking up, sketch continues from this point.
    sleep_disable();*/
    
    /*if (BT.available())
      Serial.write(BT.read());
      
    if (Serial.available())
        BT.write(Serial.read());*/
}
