/**
 * Book scanner HX711 pressure stop support.
 */

#include "../inc/MarlinConfig.h"

#if ENABLED(BOOK_SCANNER_HX711)

#include "book_scanner_hx711.h"

#include "../module/motion.h"

#include <Arduino.h>

#ifndef BOOK_SCANNER_HX711_LEFT_DOUT_PIN
  #define BOOK_SCANNER_HX711_LEFT_DOUT_PIN PG12
#endif

#ifndef BOOK_SCANNER_HX711_LEFT_SCK_PIN
  #define BOOK_SCANNER_HX711_LEFT_SCK_PIN PG13
#endif

#ifndef BOOK_SCANNER_HX711_RIGHT_DOUT_PIN
  #define BOOK_SCANNER_HX711_RIGHT_DOUT_PIN PG14
#endif

#ifndef BOOK_SCANNER_HX711_RIGHT_SCK_PIN
  #define BOOK_SCANNER_HX711_RIGHT_SCK_PIN PG15
#endif

#ifndef BOOK_SCANNER_HX711_TARE_SAMPLES
  #define BOOK_SCANNER_HX711_TARE_SAMPLES 10
#endif

#ifndef BOOK_SCANNER_HX711_THRESHOLD
  #define BOOK_SCANNER_HX711_THRESHOLD 500000L
#endif

#ifndef BOOK_SCANNER_HX711_RELEASE_THRESHOLD
  #define BOOK_SCANNER_HX711_RELEASE_THRESHOLD (BOOK_SCANNER_HX711_THRESHOLD / 2)
#endif

#ifndef BOOK_SCANNER_HX711_POLL_MS
  #define BOOK_SCANNER_HX711_POLL_MS 20
#endif

static int32_t book_scanner_hx711_tare[2] = { 0, 0 };
static int64_t book_scanner_hx711_tare_sum[2] = { 0, 0 };
static uint8_t book_scanner_hx711_tare_count[2] = { 0, 0 };
static bool book_scanner_hx711_ready[2] = { false, false };
static bool book_scanner_hx711_stopped = false;
static bool book_scanner_hx711_stop_enabled = true;
static int32_t book_scanner_hx711_delta[2] = { 0, 0 };
static millis_t book_scanner_hx711_next_ms = 0;

void BookScannerHX711::init() {
  SET_INPUT(BOOK_SCANNER_HX711_LEFT_DOUT_PIN);
  SET_INPUT(BOOK_SCANNER_HX711_RIGHT_DOUT_PIN);
  OUT_WRITE(BOOK_SCANNER_HX711_LEFT_SCK_PIN, LOW);
  OUT_WRITE(BOOK_SCANNER_HX711_RIGHT_SCK_PIN, LOW);
  SERIAL_ECHOLNPGM("BookScanner HX711 ready threshold=", BOOK_SCANNER_HX711_THRESHOLD);
}

void BookScannerHX711::idle() {
  const millis_t ms = millis();
  if (PENDING(ms, book_scanner_hx711_next_ms)) return;
  book_scanner_hx711_next_ms = ms + BOOK_SCANNER_HX711_POLL_MS;

  int32_t raw;
  if (read_sensor(BOOK_SCANNER_HX711_LEFT_DOUT_PIN, BOOK_SCANNER_HX711_LEFT_SCK_PIN, raw))
    process_sample(0, raw);
  if (read_sensor(BOOK_SCANNER_HX711_RIGHT_DOUT_PIN, BOOK_SCANNER_HX711_RIGHT_SCK_PIN, raw))
    process_sample(1, raw);
}

bool BookScannerHX711::read_sensor(const pin_t dout_pin, const pin_t sck_pin, int32_t &value) {
  if (READ(dout_pin) != LOW) return false;

  uint32_t raw = 0;
  for (uint8_t i = 0; i < 24; ++i) {
    WRITE(sck_pin, HIGH);
    delayMicroseconds(1);
    raw = (raw << 1) | (READ(dout_pin) ? 1UL : 0UL);
    WRITE(sck_pin, LOW);
    delayMicroseconds(1);
  }

  // One extra pulse selects channel A, gain 128 for the next conversion.
  WRITE(sck_pin, HIGH);
  delayMicroseconds(1);
  WRITE(sck_pin, LOW);
  delayMicroseconds(1);

  if (raw & 0x800000UL) raw |= 0xFF000000UL;
  value = int32_t(raw);
  return true;
}

void BookScannerHX711::process_sample(const uint8_t sensor, const int32_t raw) {
  if (!book_scanner_hx711_ready[sensor]) {
    book_scanner_hx711_tare_sum[sensor] += raw;
    if (++book_scanner_hx711_tare_count[sensor] >= BOOK_SCANNER_HX711_TARE_SAMPLES) {
      book_scanner_hx711_tare[sensor] = int32_t(book_scanner_hx711_tare_sum[sensor] / BOOK_SCANNER_HX711_TARE_SAMPLES);
      book_scanner_hx711_ready[sensor] = true;
      SERIAL_ECHOLNPGM("BookScanner HX711 tare sensor=", sensor, " offset=", book_scanner_hx711_tare[sensor]);
    }
    return;
  }

  const int32_t delta = ABS(raw - book_scanner_hx711_tare[sensor]);
  book_scanner_hx711_delta[sensor] = delta;
  if (book_scanner_hx711_stop_enabled && delta >= BOOK_SCANNER_HX711_THRESHOLD) {
    pressure_stop(sensor, raw, delta);
    return;
  }

  if (book_scanner_hx711_stopped && delta <= BOOK_SCANNER_HX711_RELEASE_THRESHOLD)
    book_scanner_hx711_stopped = false;
}

void BookScannerHX711::pressure_stop(const uint8_t sensor, const int32_t raw, const int32_t delta) {
  if (book_scanner_hx711_stopped) return;
  book_scanner_hx711_stopped = true;

  quickstop_stepper();
  SERIAL_ECHOLNPGM("BookScanner pressure stop sensor=", sensor, " raw=", raw, " delta=", delta);
  SERIAL_ECHOLNPGM("echo:busy: processing");
}

bool BookScannerHX711::ready() {
  return book_scanner_hx711_ready[0] && book_scanner_hx711_ready[1];
}

bool BookScannerHX711::pressure_active() {
  return ready() && (
    book_scanner_hx711_delta[0] >= BOOK_SCANNER_HX711_THRESHOLD ||
    book_scanner_hx711_delta[1] >= BOOK_SCANNER_HX711_THRESHOLD
  );
}

int32_t BookScannerHX711::last_delta(const uint8_t sensor) {
  return sensor < 2 ? book_scanner_hx711_delta[sensor] : 0;
}

void BookScannerHX711::clear_pressure_stop() {
  book_scanner_hx711_stopped = false;
}

void BookScannerHX711::set_pressure_stop_enabled(const bool enabled) {
  book_scanner_hx711_stop_enabled = enabled;
}

#endif // BOOK_SCANNER_HX711
