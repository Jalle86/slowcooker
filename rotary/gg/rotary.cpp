#include "rotary.h"

// enable pin change interrupt
static void enable_interrupt(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));
    PCIFR  |= bit (digitalPinToPCICRbit(pin));
    PCICR  |= bit (digitalPinToPCICRbit(pin));
}

RotaryEncoder::RotaryEncoder(byte d1, byte d2, byte sw, isr_func ccwf,
        isr_func cwf, isr_func buttonf)
{
    rotating        = false;
    button_down     = false;

    this->d1         = d1;
    this->d2         = d2;
    this->sw         = sw;
    this->ccwf       = ccwf;
    this->cwf        = cwf;
    this->buttonf    = buttonf;
}

void RotaryEncoder::begin(void)
{
    pinMode(d1, INPUT);
    pinMode(d2, INPUT);
    pinMode(sw, INPUT);

    enable_interrupt(d1);
    enable_interrupt(d2);
    enable_interrupt(sw);
}

void RotaryEncoder::update(void)
{
  int sws = digitalRead(sw);
  int d1s = digitalRead(this->d1);
  int d2s = digitalRead(this->d2);
  
  if (!button_down && sws == LOW)
  {
    buttonf();
    button_down = true;
  }
  else if (button_down && sws == HIGH)
    button_down = false;
  
  if (!rotating && (d1s == LOW || d2s == LOW))
  {
    rotating = true;
    if (d1s == LOW)
      ccwf();
     else
      cwf();
  }
  else if (rotating && (d1s == HIGH && d2s == HIGH))
    rotating = false;
}
