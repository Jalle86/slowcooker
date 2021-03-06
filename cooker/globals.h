#ifndef GLOBALS_H
#define GLOBALS_H

#include <stdint.h>

#define SSID_LEN    20
#define PW_LEN      20
#define AUTH_LEN    20
#define IP_LEN      16

#define TEMP_MAX    95
#define TEMP_MIN    0

extern int32_t temperature;
extern int32_t target;
extern int32_t timer;
extern int32_t activated;

extern int kp;
extern int ki;
extern int kd;

extern char esp_ip[IP_LEN];
extern char auth_phrase[AUTH_LEN];
extern char ssid[SSID_LEN];
extern char password[PW_LEN];

#endif //GLOBALS_H
