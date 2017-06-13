#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include "globals.h"

// coordinates on LCD display
#define TIMER_X 1
#define TIMER_Y 0
#define TEMP_X 1
#define TEMP_Y 1
#define LCD_W  16
#define LCD_H  2

#define CLOCK byte(0)
#define SENSOR byte(1)
#define ARROW byte(2)

enum direction
{
    COUNTERCLOCKWISE,
    CLOCKWISE,
};

typedef void (*mode_func)(enum direction);
typedef struct lcd_state (*state_func)(void);

struct lcd_state
{
    void (*init)(void); //called when state initialized
    void (*mode_func)(enum direction); // called on rotary encoder
    void (*update)(void); // called every update cycle
    struct lcd_state (*state_func)(void); //next state, called on button press
};

class LiquidCrystal;

extern byte clock[8];
extern byte tmp_sprite[8];
extern byte arrow[8];

extern struct lcd_state current_state;
extern LiquidCrystal lcd;

#endif //DISPLAY_H
