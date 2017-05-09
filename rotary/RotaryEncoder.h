#ifndef ROTARY_H
#define ROTARY_H

#include "Arduino.h"

enum encoder_state
{
    UP,
    ROTATING,
    DOWN,
};

typedef void (*cb_func)(void);

class RotaryEncoder
{
    private:
    byte d1, d2, sw;
    enum encoder_state state;
    bool button_down;
    cb_func ccwcb; // counter clockwise
    cb_func cwcb; // clockwise
    cb_func buttoncb;

    public:
    RotaryEncoder(byte d1, byte d2, byte sw, isr_func ccwf,
        isr_func cwf, isr_func buttonf);
    void begin(void);
    void update(void);
};

#endif //ROTARY_H
