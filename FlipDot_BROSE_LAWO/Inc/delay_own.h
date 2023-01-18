#include "stm32f1xx_hal.h"

// Provisorisches µS Delay (Funktioniert bei 72Mhz ganz okay)
void delay_us(uint32_t microseconds) {

  uint32_t cycles = 4.65 * microseconds;
  for (volatile int i = 0; i < cycles; i++) {}
}