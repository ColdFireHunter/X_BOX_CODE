#ifndef _global_h
#define _global_h

#include <Arduino.h>
#include <pins.h>

extern HardwareSerial DEBUG_SERIAL;
extern HardwareSerial EXPANSIION_SERIAL;
extern volatile uint16_t adc_values[6];

#define ADC_CONVERT_TIME 250

#endif // _global_h