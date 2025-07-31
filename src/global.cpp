#include <global.h>

HardwareSerial DEBUG_SERIAL(DEBUG_RX, DEBUG_TX);
HardwareSerial EXPANSION_SERIAL(EXPANSION_RX, EXPANSION_TX);

volatile uint16_t adc_values[6];