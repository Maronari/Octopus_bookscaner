/**
 * Book scanner UART remote support.
 */
#pragma once

#include "../inc/MarlinConfigPre.h"

#if ENABLED(BOOK_SCANNER_REMOTE)

class BookScannerRemote {
public:
  static void init();
  static void idle();

private:
  static void handle_command(const char c);
  static void inject_move(const char axis, const int8_t dir, const float mm, const uint16_t feedrate);
  static void set_light_level(const uint8_t pwm);
  static void cycle_light();
};

#endif
