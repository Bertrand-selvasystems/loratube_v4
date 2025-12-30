#pragma once
#include <stdint.h>

typedef struct {
    uint16_t year;   // 2000..2099
    uint8_t  month;  // 1..12
    uint8_t  day;    // 1..31
    uint8_t  hour;   // 0..23
    uint8_t  min;    // 0..59
    uint8_t  sec;    // 0..59
    uint8_t  wday;   // 0..6 (optionnel mais souvent utile)
} rtc_time_msg_t;