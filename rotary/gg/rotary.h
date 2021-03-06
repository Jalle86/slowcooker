#ifndef ROTARY_H
#define ROTARY_H

#include "Arduino.h"

typedef void (*isr_func)(void);

class RotaryEncoder
{
    private:
    byte d1, d2, sw;
    bool rotating;
    bool button_down;
    isr_func ccwf, cwf, buttonf;

    public:
    RotaryEncoder(byte d1, byte d2, byte sw, isr_func ccwf,
        isr_func cwf, isr_func buttonf);
    void begin(void);
    void update(void);
};

#endif //ROTARY_H
