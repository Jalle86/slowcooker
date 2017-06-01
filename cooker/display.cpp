#include "display.h"
#include <LiquidCrystal.h>

static void view_init(void);
static void view_update(void);
static void view_change(enum direction d);
static struct lcd_mode view_next(void);

static void menu_init(void);
static void menu_update(void);
static void display_cursor(void);
static void menu_change(enum direction d);
static struct lcd_mode menu_next(void);

static void opt_trgt_init(void);
static void opt_tmr_init(void);
static void opt_trgt_update(void);
static void opt_tmr_update(void);
static void target_change(enum direction d);
static void timer_change(enum direction d);
static struct lcd_mode option_next(void);

static void print_time(int col, int row, int val);
static void print_timer(int col, int row);
static void print_target(int col, int row);
static void print_temperature(int col, int row);

struct option
{
	char *menu_text;
	char *option_text;
	struct lcd_mode next;
};

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
	B11110,
	B11100,
	B11000,
	B10000,
	B00000,
};

static struct lcd_mode mode[] =
{
	{ view_init,      view_change,    view_update, view_next   },
	{ menu_init,      menu_change,    menu_update, menu_next   },
	{ opt_trgt_init,  target_change,  opt_trgt_update, option_next },
	{ opt_tmr_init,   timer_change,   opt_tmr_update, option_next },
};
struct lcd_mode current_mode = mode[0];

static struct option options[] =
{
	{ "Target", "Target temp.:", mode[2] },
	{ "Timer", "Timer:", mode[3] },
};
static int menu_position = 0;


static void view_init(void)
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
  
  print_temperature(TEMP_X, TEMP_Y);
  print_timer(TIMER_X, TIMER_Y);
}

static void view_update(void)
{
  static int temp_old = temperature;
  static int timer_old = timer;
  
  if (temperature != temp_old)
  {
    print_temperature(TEMP_X, TEMP_Y);
    temp_old = temperature;
  }
    
  if (timer != timer_old)
  {
    print_timer(TIMER_X, TIMER_Y);
    timer_old = timer;
  }
}

// dummy function
static void view_change(enum direction d)
{
}

static struct lcd_mode view_next(void)
{
  return mode[1];
}

// MENU //

#define SET_MENU_POS(X) (lcd.setCursor(1 + 7 * ((X) % 2), (X) / 2))
static void menu_init(void)
{
  int i = 0;
  menu_position = 0;
  
  for (i = 0; i < sizeof(options)/sizeof(*options); i++)
  {
    SET_MENU_POS(i);
    lcd.print(options[i].menu_text);
  }
  
  SET_MENU_POS(i);
  lcd.print("Back");
  
  display_cursor();
}

static void menu_update(void)
{
}

static void display_cursor(void)
{
  lcd.setCursor(7 * (menu_position % 2), menu_position / 2);
  lcd.write(ARROW);
}

static void menu_change(enum direction d)
{
  lcd.setCursor(7 * (menu_position % 2), menu_position / 2);
  lcd.write(' '); //clear cursor
  
  if (d == COUNTERCLOCKWISE)
    menu_position--;
  else //CLOCKWISE
    menu_position++;
    
  if (menu_position < 0)
     menu_position = sizeof(options) / sizeof(*options);
  else if (menu_position > sizeof(options)/sizeof(*options))
    menu_position = 0;
    
  display_cursor();
}

static struct lcd_mode menu_next(void)
{
  if (menu_position < sizeof(options)/sizeof(*options))
    return options[menu_position].next;
  else
    return mode[0];
}


// OPTION //

static void opt_trgt_init(void)
{
  lcd.setCursor(0, 0);
  lcd.print("Target temp.:");
  print_target(0, 1);
  lcd.write(0xDF); //degree sign
  lcd.write('C');  
}

static void opt_tmr_init(void)
{
  lcd.setCursor(0, 0);
  lcd.print("Timer:");
  lcd.setCursor(0, 1);
  
  print_timer(0, 1);
}

static void opt_trgt_update(void)
{
  static int target_old = target;
  
  if (target != target_old)
  {
    print_target(0, 1);
    target_old = target;
  }
}

static void opt_tmr_update(void)
{
  static int timer_old = timer;
  
  if (timer != timer_old)
  {
    print_timer(0, 1);
    timer_old = timer;
  }
}

static void target_change(enum direction d)
{
  if (d == CLOCKWISE)
    target = (target == TEMP_MAX) ? TEMP_MAX : target + 1;
  else
    target = (target == TEMP_MIN) ? TEMP_MIN : target - 1;
}

static void timer_change(enum direction d)
{
  int minutes = (timer % 3600) / 60;
  if (d == CLOCKWISE)
    timer += 5 * 60; //5 minutes increment
  else if (d == COUNTERCLOCKWISE && minutes >= 5)
    timer -= 5 * 60;
}

static struct lcd_mode option_next(void)
{
  return mode[0];
}
    
static void print_time(int col, int row, int val)
{
    char num_string[3] = { 0 };
    sprintf(num_string, "%02d", val);
    
    lcd.setCursor(col, row);
    lcd.print(num_string);
}

static void print_timer(int col, int row)
{
  int hours, minutes;
  uint32_t tmp = timer;

  // hours
  hours = tmp / 3600;
  print_time(col, row, hours);
  lcd.write(':');

  // minutes
  tmp %= 3600;
  minutes = tmp / 60;
  print_time(col + 3, row, minutes);
  lcd.write(':');
  
  // seconds
  tmp %= 60;
  print_time(col + 6, row, tmp);
}

static void print_target(int col, int row)
{
  lcd.setCursor(col, row);
  lcd.print("  "); //erase previous entry
  lcd.setCursor(col, row);
  lcd.print(target);
}

static void print_temperature(int col, int row)
{
  char str[5];
  //format "xx.x" degrees
  int t = temperature; //sprintf doesn't take kindly to int32_t
  sprintf(str, "%02d.%d", t / 10, t % 10);
  
  lcd.setCursor(col, row);
  lcd.print(str);
}
