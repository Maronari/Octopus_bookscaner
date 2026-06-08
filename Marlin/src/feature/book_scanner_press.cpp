/**
 * Book scanner Z press cycle.
 *
 * M932 - slow clamp down until HX711 pressure threshold
 * M933 - fast release up with a parabolic speed profile
 */

#include "../inc/MarlinConfig.h"

#if ENABLED(BOOK_SCANNER_PRESS)

#include "book_scanner_press.h"

#include "../module/motion.h"
#include "../module/planner.h"

#if ENABLED(BOOK_SCANNER_HX711)
  #include "book_scanner_hx711.h"
#endif

#include <Arduino.h>

#ifndef BOOK_SCANNER_PRESS_CLAMP_MM
  #define BOOK_SCANNER_PRESS_CLAMP_MM 300.0f
#endif

#ifndef BOOK_SCANNER_PRESS_RELEASE_MM
  #define BOOK_SCANNER_PRESS_RELEASE_MM 300.0f
#endif

#ifndef BOOK_SCANNER_PRESS_STEP_MM
  #define BOOK_SCANNER_PRESS_STEP_MM 1.0f
#endif

#ifndef BOOK_SCANNER_PRESS_CLAMP_FEEDRATE_MM_M
  #define BOOK_SCANNER_PRESS_CLAMP_FEEDRATE_MM_M 240
#endif

#ifndef BOOK_SCANNER_PRESS_RELEASE_MIN_FEEDRATE_MM_M
  #define BOOK_SCANNER_PRESS_RELEASE_MIN_FEEDRATE_MM_M 600
#endif

#ifndef BOOK_SCANNER_PRESS_RELEASE_MAX_FEEDRATE_MM_M
  #define BOOK_SCANNER_PRESS_RELEASE_MAX_FEEDRATE_MM_M 3600
#endif

#ifndef BOOK_SCANNER_PRESS_RELEASE_SEGMENTS
  #define BOOK_SCANNER_PRESS_RELEASE_SEGMENTS 30
#endif

static void sync_z_from_stepper() {
  planner.synchronize();
  set_current_from_steppers_for_axis(Z_AXIS);
  sync_plan_position();
}

static void move_z_delta(const float delta, const feedRate_t feedrate) {
  current_position.z += delta;
  line_to_current_position(feedrate);
  sync_z_from_stepper();
}

void BookScannerPress::clamp() {
  SERIAL_ECHOLNPGM("BookScanner clamp start");

  #if ENABLED(BOOK_SCANNER_HX711)
    if (!BookScannerHX711::ready()) {
      SERIAL_ECHOLNPGM("BookScanner clamp blocked: HX711 not ready");
      return;
    }
    BookScannerHX711::set_pressure_stop_enabled(true);
    BookScannerHX711::clear_pressure_stop();
  #endif

  const feedRate_t fr = MMM_TO_MMS(BOOK_SCANNER_PRESS_CLAMP_FEEDRATE_MM_M);
  float moved = 0;

  while (moved < BOOK_SCANNER_PRESS_CLAMP_MM) {
    #if ENABLED(BOOK_SCANNER_HX711)
      if (BookScannerHX711::pressure_active()) {
        SERIAL_ECHOLNPGM(
          "BookScanner clamp pressure reached left=", BookScannerHX711::last_delta(0),
          " right=", BookScannerHX711::last_delta(1)
        );
        return;
      }
    #endif

    const float step = _MIN(BOOK_SCANNER_PRESS_STEP_MM, BOOK_SCANNER_PRESS_CLAMP_MM - moved);
    move_z_delta(-step, fr);
    moved += step;
  }

  SERIAL_ECHOLNPGM("BookScanner clamp max travel reached");
}

void BookScannerPress::release() {
  SERIAL_ECHOLNPGM("BookScanner release start");

  #if ENABLED(BOOK_SCANNER_HX711)
    BookScannerHX711::set_pressure_stop_enabled(false);
    BookScannerHX711::clear_pressure_stop();
  #endif

  constexpr uint8_t segments = _MAX(uint8_t(2), uint8_t(BOOK_SCANNER_PRESS_RELEASE_SEGMENTS));
  const float segment_mm = BOOK_SCANNER_PRESS_RELEASE_MM / segments;
  const float min_fr = BOOK_SCANNER_PRESS_RELEASE_MIN_FEEDRATE_MM_M;
  const float max_fr = BOOK_SCANNER_PRESS_RELEASE_MAX_FEEDRATE_MM_M;

  for (uint8_t i = 0; i < segments; ++i) {
    const float t = (float(i) + 0.5f) / float(segments);
    const float parabola = 4.0f * t * (1.0f - t);
    const float fr_mm_m = min_fr + (max_fr - min_fr) * parabola;
    move_z_delta(segment_mm, MMM_TO_MMS(fr_mm_m));
  }

  SERIAL_ECHOLNPGM("BookScanner release done");

  #if ENABLED(BOOK_SCANNER_HX711)
    BookScannerHX711::clear_pressure_stop();
    BookScannerHX711::set_pressure_stop_enabled(true);
  #endif
}

#endif // BOOK_SCANNER_PRESS
