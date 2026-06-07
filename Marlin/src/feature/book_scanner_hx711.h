/**
 * Book scanner HX711 pressure stop support.
 */
#pragma once

#include "../inc/MarlinConfigPre.h"

#if ENABLED(BOOK_SCANNER_HX711)

class BookScannerHX711 {
public:
  static void init();
  static void idle();

private:
  static bool read_sensor(const pin_t dout_pin, const pin_t sck_pin, int32_t &value);
  static void process_sample(const uint8_t sensor, const int32_t raw);
  static void pressure_stop(const uint8_t sensor, const int32_t raw, const int32_t delta);
};

#endif
