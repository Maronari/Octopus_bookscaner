/**
 * Book scanner UART remote support.
 *
 * Remote commands are single ASCII digits:
 * 0 scan event, 1 light cycle, 2..7 motion, 8 stop, 9 homing test.
 */

#include "../inc/MarlinConfig.h"

#if ENABLED(BOOK_SCANNER_REMOTE)

#include "book_scanner_remote.h"

#include "../gcode/queue.h"

#include <Arduino.h>

#ifndef BOOK_SCANNER_REMOTE_BAUDRATE
  #define BOOK_SCANNER_REMOTE_BAUDRATE 115200
#endif

#ifndef BOOK_SCANNER_REMOTE_RX_PIN
  #define BOOK_SCANNER_REMOTE_RX_PIN PD9
#endif

#ifndef BOOK_SCANNER_REMOTE_TX_PIN
  #define BOOK_SCANNER_REMOTE_TX_PIN PD8
#endif

#ifndef BOOK_SCANNER_REMOTE_XY_MM
  #define BOOK_SCANNER_REMOTE_XY_MM 2.0f
#endif

#ifndef BOOK_SCANNER_REMOTE_Z_MM
  #define BOOK_SCANNER_REMOTE_Z_MM 300.0f
#endif

#ifndef BOOK_SCANNER_REMOTE_XY_FEEDRATE
  #define BOOK_SCANNER_REMOTE_XY_FEEDRATE 1200
#endif

#ifndef BOOK_SCANNER_REMOTE_Z_FEEDRATE
  #define BOOK_SCANNER_REMOTE_Z_FEEDRATE 480
#endif

#ifndef BOOK_SCANNER_REMOTE_LIGHT_FAN_1
  #define BOOK_SCANNER_REMOTE_LIGHT_FAN_1 0
#endif

#ifndef BOOK_SCANNER_REMOTE_LIGHT_FAN_2
  #define BOOK_SCANNER_REMOTE_LIGHT_FAN_2 1
#endif

static HardwareSerial book_scanner_remote_serial(BOOK_SCANNER_REMOTE_RX_PIN, BOOK_SCANNER_REMOTE_TX_PIN);
static uint8_t book_scanner_light_step = 0;

void BookScannerRemote::init() {
  book_scanner_remote_serial.begin(BOOK_SCANNER_REMOTE_BAUDRATE);
  SERIAL_ECHOLNPGM("BookScanner remote UART ready baud=", BOOK_SCANNER_REMOTE_BAUDRATE);
}

void BookScannerRemote::idle() {
  while (book_scanner_remote_serial.available() > 0) {
    const int value = book_scanner_remote_serial.read();
    if (value >= 0) handle_command(char(value));
  }
}

void BookScannerRemote::handle_command(const char c) {
  if (c < '0' || c > '9') return;

  SERIAL_ECHOLNPGM("BookScanner remote command=", c);

  switch (c) {
    case '0':
      queue.inject(F("M118 A1 action:scan"));
      break;

    case '1':
      cycle_light();
      break;

    case '2': // right carriage down
      inject_move('Y', -1, BOOK_SCANNER_REMOTE_XY_MM, BOOK_SCANNER_REMOTE_XY_FEEDRATE);
      break;

    case '3': // right carriage up
      inject_move('Y', 1, BOOK_SCANNER_REMOTE_XY_MM, BOOK_SCANNER_REMOTE_XY_FEEDRATE);
      break;

    case '4': // left carriage down
      inject_move('X', -1, BOOK_SCANNER_REMOTE_XY_MM, BOOK_SCANNER_REMOTE_XY_FEEDRATE);
      break;

    case '5': // left carriage up
      inject_move('X', 1, BOOK_SCANNER_REMOTE_XY_MM, BOOK_SCANNER_REMOTE_XY_FEEDRATE);
      break;

    case '6': // center carriage down
      inject_move('Z', -1, BOOK_SCANNER_REMOTE_Z_MM, BOOK_SCANNER_REMOTE_Z_FEEDRATE);
      break;

    case '7': // center carriage up
      inject_move('Z', 1, BOOK_SCANNER_REMOTE_Z_MM, BOOK_SCANNER_REMOTE_Z_FEEDRATE);
      break;

    case '8':
      queue.inject(F("M410\nM400"));
      break;

    case '9':
      queue.inject(F("G28 X\nG28 Y\nG28 Z"));
      break;
  }
}

void BookScannerRemote::inject_move(const char axis, const int8_t dir, const float mm, const uint16_t feedrate) {
  char cmd[64];
  const int32_t centi_mm = int32_t(mm * 100.0f + 0.5f) * dir;
  const char sign = centi_mm < 0 ? '-' : '\0';
  const uint32_t abs_centi_mm = uint32_t(centi_mm < 0 ? -centi_mm : centi_mm);
  if (sign)
    snprintf_P(cmd, sizeof(cmd), PSTR("M120\nG91\nG1 %c-%lu.%02lu F%u\nG90"), axis, (unsigned long)(abs_centi_mm / 100), (unsigned long)(abs_centi_mm % 100), feedrate);
  else
    snprintf_P(cmd, sizeof(cmd), PSTR("M120\nG91\nG1 %c%lu.%02lu F%u\nG90"), axis, (unsigned long)(abs_centi_mm / 100), (unsigned long)(abs_centi_mm % 100), feedrate);
  queue.inject(cmd);
}

void BookScannerRemote::set_light_level(const uint8_t pwm) {
  char cmd[40];
  snprintf_P(cmd, sizeof(cmd), PSTR("M106 P%d S%d\nM106 P%d S%d"),
    BOOK_SCANNER_REMOTE_LIGHT_FAN_1, pwm,
    BOOK_SCANNER_REMOTE_LIGHT_FAN_2, pwm
  );
  queue.inject(cmd);
  SERIAL_ECHOLNPGM("BookScanner light pwm=", pwm);
}

void BookScannerRemote::cycle_light() {
  static constexpr uint8_t levels[] = { 0, 51, 255 };
  book_scanner_light_step = (book_scanner_light_step + 1) % 3;
  set_light_level(levels[book_scanner_light_step]);
}

#endif // BOOK_SCANNER_REMOTE
