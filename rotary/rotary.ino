#define D1      11
#define D2      12
#define SWITCH  13

#define LOW     0
#define HIGH    1

#include <string.h>

int state = LOW;
int rotating = 0;

void setup(void)
{
  pinMode(D1, INPUT_PULLUP);
  pinMode(D2, INPUT);
  pinMode(SWITCH, INPUT);
  
  Serial.begin(9600);
  
  enable_interrupt(D1);
  enable_interrupt(D2);
  enable_interrupt(SWITCH);
}

void enable_interrupt(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));
    PCIFR  |= bit (digitalPinToPCICRbit(pin));
    PCICR  |= bit (digitalPinToPCICRbit(pin));
}

void print_int(int n)
{
  char str[10];
  sprintf(str, "%d", n);
  Serial.write(str);
}

ISR (PCINT0_vect)
{
  int s = digitalRead(SWITCH);
  int d1 = digitalRead(D1);
  int d2 = digitalRead(D2);
  
  if (state == LOW && s == 0)
  {
    Serial.write("Button down\n");
    state = HIGH;
  }
  else if (state == HIGH && s == 1)
  {
    Serial.write("Button up\n");
    state = LOW;
  }
  
  if (!rotating && (d1 == 0 || d2 == 0))
  {
    rotating = 1;
    if (d1 == 0)
      Serial.write("CW\n");
     else
       Serial.write("CCW\n");
  }
  else if (rotating && (d1 == 1 && d2 == 1))
  {
    rotating = 0;
    Serial.write("stopped rotating\n");
  }
}

void loop(void)
{  
  while(1)
  {
    /*print_int(digitalRead(D1));
    Serial.write(" ");
    print_int(digitalRead(D2));
    Serial.write(" ");
    print_int(digitalRead(SWITCH));
    Serial.write("\n");*/
  }
}


