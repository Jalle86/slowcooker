#ifndef VARS_H
#define VARS_H

#include <stdint.h>

#define SSID_LEN 20
#define PW_LEN 20

extern int32_t temperature;
extern int32_t target;
extern int32_t timer;
extern int32_t activated;

extern char esp_ip[16];
extern char ssid[SSID_LEN];
extern char password[PW_LEN];

#endif
