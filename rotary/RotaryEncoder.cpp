#include "RotaryEncoder.h"

// enable pin change interrupt
static void pci_setup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));
    PCIFR  |= bit (digitalPinToPCICRbit(pin));
    PCICR  |= bit (digitalPinToPCICRbit(pin));
}

RotaryEncoder::RotaryEncoder(byte d1, byte d2, byte sw, isr_func ccwf,
        isr_func cwf, isr_func buttonf)
{
    state           = UP;
    button_down     = false;

    this->d1         = d1;
    this->d2         = d2;
    this->sw         = sw;
    this->ccwcb      = ccwf;
    this->cwcb       = cwf;
    this->buttoncb   = buttonf;
}

void RotaryEncoder::begin(void)
{
    pinMode(d1, INPUT);
    pinMode(d2, INPUT);
    pinMode(sw, INPUT);

    // whenever there's a change in input, trigger an interrupt
    pci_setup(d1);
    pci_setup(d2);
    pci_setup(sw);
}

// This method should be called in an ISR
// Updates the state of the encoder and calls the appropriate
// callback functions if events (rotate cw, rotate ccw, button down)
// are triggered
void RotaryEncoder::update(void)
{
  int sws = digitalRead(sw);
  int d1s = digitalRead(this->d1);
  int d2s = digitalRead(this->d2);
  
  if (!button_down && sws == LOW)
  {
    buttoncb();
    button_down = true;
  }
  else if (sws == HIGH)
    button_down = false;
  
  switch(state)
  {
  case UP:
    if (!d1s || !d2s)
    {
      state = ROTATING;

      // appropriate rotation direction depends on whether D1 or D2
      // is the first to hit LOW state
      d1s ? cwcb() : ccwcb();
    }
    break;

  case ROTATING:
    if (!d1s && !d2s)
      state = DOWN;
    break;

  case DOWN:
    if (d1s && d2s)
      state = UP; // return to original state
  }
}
