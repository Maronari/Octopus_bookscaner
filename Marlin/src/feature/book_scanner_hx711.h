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
  static bool ready();
  static bool pressure_active();
  static int32_t last_delta(const uint8_t sensor);
  static void clear_pressure_stop();
  static void set_pressure_stop_enabled(const bool enabled);

private:
  static bool read_sensor(const pin_t dout_pin, const pin_t sck_pin, int32_t &value);
  static void process_sample(const uint8_t sensor, const int32_t raw);
  static void pressure_stop(const uint8_t sensor, const int32_t raw, const int32_t delta);
};

#endif
