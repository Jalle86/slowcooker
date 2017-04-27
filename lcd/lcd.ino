#include <LiquidCrystal.h>

#define DB4      5
#define DB5      4
#define DB6      3
#define DB7      2

#define RS       6
#define RW       7
#define ENABLE   8

#define READ     1
#define WRITE    0

#define COMMAND  0
#define DATA     1

void wait_busy()
{
  pinMode(DB7, INPUT);
  while(digitalRead(DB7));
  pinMode(DB7, OUTPUT);
}

void send_byte(byte b)
{
  Serial.write("byte\n");
  char c[10];
  send_nibble(b >> 4);
  sprintf(c, "%d\n", b >> 4);
  Serial.write(c);
  
  send_nibble(b & 0x0F);
  sprintf(c, "%d\n", b & 0x0F);
  Serial.write(c);
}

void send_nibble(byte nib)
{
  char c[10];
  Serial.write("nibble ");
  digitalWrite(ENABLE, HIGH);
  
  digitalWrite(DB7, !!(nib & 0x8));
  sprintf(c, "%d", digitalRead(DB7));
  Serial.write(c);
  
  digitalWrite(DB6, !!(nib & 0x4));
  sprintf(c, "%d", digitalRead(DB6));
  Serial.write(c);
  
  digitalWrite(DB5, !!(nib & 0x2));
  sprintf(c, "%d", digitalRead(DB5));
  Serial.write(c);
  
  digitalWrite(DB4, !!(nib & 0x1));
  sprintf(c, "%d\n", digitalRead(DB4));
  Serial.write(c);
  
  delayMicroseconds(1);
  
  digitalWrite(ENABLE, LOW);
  delayMicroseconds(1);
}

void setup(void)
{
  Serial.begin(9600);
  Serial.write("begin\n");
  
  pinMode(DB4, OUTPUT);
  pinMode(DB5, OUTPUT);
  pinMode(DB6, OUTPUT);
  pinMode(DB7, OUTPUT);
  pinMode(RS, OUTPUT);
  pinMode(RW, OUTPUT);
  pinMode(ENABLE, OUTPUT);
  
  digitalWrite(DB4, LOW);
  digitalWrite(DB5, LOW);
  digitalWrite(DB6, LOW);
  digitalWrite(DB7, LOW);
  digitalWrite(RS, COMMAND);
  digitalWrite(RW, WRITE);
  digitalWrite(ENABLE, LOW);
  
  // 8 bit mode, ificantlower 4 bits insign
  delay(40);
  send_nibble(0x03);
  delay(200);
  send_nibble(0x03);
  delay(200);
  send_nibble(0x03);
  delay(200);
  send_nibble(0x02);
  delay(200);
  
  //4 bit mode 
  
  delay(200);
  send_byte(0x28); //function set
  
  delay(200);
  send_byte(0x08); //display, cursor, blink off
  
  delay(200);
  send_byte(0x01); //clear screen
  
  delay(200);
  send_byte(0x0F); //display on
  
  delay(200);
  send_byte(0x06); //inc cursor
  
  delay(200);
  send_byte(0x80); //cursor first pos
}

void loop(void)
{
}
