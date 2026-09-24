/*
 * Config.h — IR line sensor test config.
 *
 * Values copied from TAXI_Firmware/Config.h so the test evaluates the
 * sensors exactly like the combat firmware does. If you change a pin or
 * threshold in the firmware, update it here too.
 */
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Line sensor pins (from TAXI_Firmware/Config.h)
#define PIN_LINE_L   A5     // QRE1113 line sensor, LEFT
#define PIN_LINE_R   A1     // QRE1113 line sensor, RIGHT
#define PIN_LINE_B   A4     // QRE1113 line sensor, REAR

// Per-sensor thresholds (from TAXI_Firmware/Config.h). Raw ADC is HIGH over
// black (interior) and LOW over white (border); edge = white = LOW reading.
#define LINE_LEFT_WHITE_THRESHOLD   400   // LEFT  raw BELOW this = white border
#define LINE_LEFT_BLACK_THRESHOLD   800   // LEFT  raw ABOVE this = black interior
#define LINE_REAR_WHITE_THRESHOLD   765   // REAR  raw BELOW this = white border
#define LINE_REAR_BLACK_THRESHOLD   820   // REAR  raw ABOVE this = black interior
#define LINE_RIGHT_WHITE_THRESHOLD  400   // RIGHT raw BELOW this = white border
#define LINE_RIGHT_BLACK_THRESHOLD  800   // RIGHT raw ABOVE this = black interior
#define LINE_DEBOUNCE_SAMPLES       2

// Serial Monitor baud (from TAXI_Firmware/Config.h)
#define MONITOR_BAUD     115200

#endif // CONFIG_H
