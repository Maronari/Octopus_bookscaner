/**
 * Book scanner startup calibration.
 *
 * M930     - calibrate X, Y and Z
 * M930 X   - calibrate X only
 * M931     - report measured axis lengths
 */

#include "../inc/MarlinConfig.h"

#if ENABLED(BOOK_SCANNER_CALIBRATION)

#include "book_scanner_calibration.h"

#include "../module/endstops.h"
#include "../module/motion.h"
#include "../module/planner.h"

#if ENABLED(EEPROM_SETTINGS)
  #include "../HAL/shared/eeprom_api.h"
#endif

#include <Arduino.h>

#ifndef BOOK_SCANNER_CALIBRATION_STEP_MM
  #define BOOK_SCANNER_CALIBRATION_STEP_MM 1.0f
#endif

#ifndef BOOK_SCANNER_CALIBRATION_BACKOFF_MM
  #define BOOK_SCANNER_CALIBRATION_BACKOFF_MM 5.0f
#endif

#ifndef BOOK_SCANNER_CALIBRATION_MAX_TRAVEL_MM
  #define BOOK_SCANNER_CALIBRATION_MAX_TRAVEL_MM 1200.0f
#endif

#ifndef BOOK_SCANNER_CALIBRATION_FEEDRATE_MM_M
  #define BOOK_SCANNER_CALIBRATION_FEEDRATE_MM_M 600
#endif

#ifndef BOOK_SCANNER_CALIBRATION_CENTER_FEEDRATE_MM_M
  #define BOOK_SCANNER_CALIBRATION_CENTER_FEEDRATE_MM_M 3000
#endif

static float book_scanner_axis_length[3] = { 0, 0, 0 };
static bool book_scanner_axis_calibrated[3] = { false, false, false };

struct BookScannerCalibrationStore {
  uint32_t magic;
  uint8_t version;
  uint8_t calibrated[3];
  float length[3];
};

static constexpr uint32_t BOOK_SCANNER_CALIBRATION_MAGIC = 0x42534341UL; // BSCA
static constexpr uint8_t BOOK_SCANNER_CALIBRATION_VERSION = 1;

static inline feedRate_t calibration_feedrate() {
  return MMM_TO_MMS(BOOK_SCANNER_CALIBRATION_FEEDRATE_MM_M);
}

static inline feedRate_t center_feedrate() {
  return MMM_TO_MMS(BOOK_SCANNER_CALIBRATION_CENTER_FEEDRATE_MM_M);
}

static bool endstop_triggered(const AxisEnum axis, const bool toward_max) {
  switch (axis) {
    case X_AXIS:
      return toward_max ? (READ(X_MAX_PIN) != X_MAX_ENDSTOP_INVERTING) : (READ(X_MIN_PIN) != X_MIN_ENDSTOP_INVERTING);
    case Y_AXIS:
      return toward_max ? (READ(Y_MAX_PIN) != Y_MAX_ENDSTOP_INVERTING) : (READ(Y_MIN_PIN) != Y_MIN_ENDSTOP_INVERTING);
    case Z_AXIS:
      return toward_max ? (READ(Z_MAX_PIN) != Z_MAX_ENDSTOP_INVERTING) : (READ(Z_MIN_PIN) != Z_MIN_ENDSTOP_INVERTING);
    default:
      return false;
  }
}

static char axis_letter(const AxisEnum axis) {
  switch (axis) {
    case X_AXIS: return 'X';
    case Y_AXIS: return 'Y';
    case Z_AXIS: return 'Z';
    default: return '?';
  }
}

static void sync_axis_from_stepper(const AxisEnum axis) {
  planner.synchronize();
  set_current_from_steppers_for_axis(axis);
  sync_plan_position();
}

static void move_axis_delta(const AxisEnum axis, const float delta, const feedRate_t feedrate) {
  current_position[axis] += delta;
  line_to_current_position(feedrate);
  sync_axis_from_stepper(axis);
}

static bool valid_axis_length(const float length) {
  return length > 0.0f && length <= BOOK_SCANNER_CALIBRATION_MAX_TRAVEL_MM;
}

static bool release_endstop(const AxisEnum axis, const bool from_max) {
  if (!endstop_triggered(axis, from_max)) return true;

  const float delta = from_max ? -BOOK_SCANNER_CALIBRATION_STEP_MM : BOOK_SCANNER_CALIBRATION_STEP_MM;
  for (float moved = 0; moved < BOOK_SCANNER_CALIBRATION_BACKOFF_MM * 4; moved += BOOK_SCANNER_CALIBRATION_STEP_MM) {
    move_axis_delta(axis, delta, calibration_feedrate());
    if (!endstop_triggered(axis, from_max)) return true;
  }

  SERIAL_ECHOLNPGM("BookScanner calibration failed: ", axis_letter(axis), " endstop did not release");
  return false;
}

static bool backoff_endstop(const AxisEnum axis, const bool from_max) {
  const float delta = from_max ? -BOOK_SCANNER_CALIBRATION_BACKOFF_MM : BOOK_SCANNER_CALIBRATION_BACKOFF_MM;
  move_axis_delta(axis, delta, calibration_feedrate());
  if (!endstop_triggered(axis, from_max)) return true;

  SERIAL_ECHOLNPGM("BookScanner calibration failed: ", axis_letter(axis), " endstop still active after backoff");
  return false;
}

static bool seek_endstop(const AxisEnum axis, const bool toward_max, float &traveled) {
  const float delta = toward_max ? BOOK_SCANNER_CALIBRATION_STEP_MM : -BOOK_SCANNER_CALIBRATION_STEP_MM;
  traveled = 0;

  for (; traveled <= BOOK_SCANNER_CALIBRATION_MAX_TRAVEL_MM; traveled += BOOK_SCANNER_CALIBRATION_STEP_MM) {
    if (endstop_triggered(axis, toward_max)) {
      sync_axis_from_stepper(axis);
      return true;
    }
    move_axis_delta(axis, delta, calibration_feedrate());
  }

  SERIAL_ECHOLNPGM("BookScanner calibration failed: ", axis_letter(axis), toward_max ? " max" : " min", " not reached");
  return false;
}

void BookScannerCalibration::init() {
  load();
  apply_limits();
  report();
}

void BookScannerCalibration::calibrate(const bool do_x, const bool do_y, const bool do_z) {
  SERIAL_ECHOLNPGM("BookScanner calibration start");
  bool ok = true;

  #if HAS_SOFTWARE_ENDSTOPS
    const bool old_soft_endstop = soft_endstop._enabled;
    soft_endstop._enabled = false;
  #endif

  if (do_x && !calibrate_axis(X_AXIS)) ok = false;
  if (do_y && !calibrate_axis(Y_AXIS)) ok = false;
  if (do_z && !calibrate_axis(Z_AXIS)) ok = false;

  #if HAS_SOFTWARE_ENDSTOPS
    soft_endstop._enabled = old_soft_endstop;
  #endif

  if (ok) {
    apply_limits();
    save();
    SERIAL_ECHOLNPGM("BookScanner calibration done");
  }
  else
    SERIAL_ECHOLNPGM("BookScanner calibration failed");

  report();
}

bool BookScannerCalibration::calibrate_axis(const AxisEnum axis) {
  const uint8_t idx = uint8_t(axis);
  SERIAL_ECHOLNPGM("BookScanner calibrating ", axis_letter(axis));

  if (!release_endstop(axis, true)) return false;
  float ignored = 0;
  if (!seek_endstop(axis, false, ignored)) return false;

  current_position[axis] = 0;
  sync_plan_position();

  if (!backoff_endstop(axis, false)) return false;
  float length = BOOK_SCANNER_CALIBRATION_BACKOFF_MM;

  float traveled = 0;
  if (!seek_endstop(axis, true, traveled)) return false;
  length += traveled;

  current_position[axis] = length;
  sync_plan_position();

  book_scanner_axis_length[idx] = length;
  book_scanner_axis_calibrated[idx] = true;

  if (!backoff_endstop(axis, true)) return false;
  sync_plan_position();
  move_axis_delta(axis, -(length * 0.5f - BOOK_SCANNER_CALIBRATION_BACKOFF_MM), center_feedrate());
  current_position[axis] = length * 0.5f;
  sync_plan_position();

  SERIAL_ECHOLNPGM("BookScanner ", axis_letter(axis), " length=", length, " center=", current_position[axis]);
  return true;
}

void BookScannerCalibration::report() {
  SERIAL_ECHOLNPGM(
    "BookScanner calibration X=", book_scanner_axis_calibrated[X_AXIS] ? book_scanner_axis_length[X_AXIS] : -1,
    " Y=", book_scanner_axis_calibrated[Y_AXIS] ? book_scanner_axis_length[Y_AXIS] : -1,
    " Z=", book_scanner_axis_calibrated[Z_AXIS] ? book_scanner_axis_length[Z_AXIS] : -1
  );
}

void BookScannerCalibration::apply_limits() {
  #if HAS_SOFTWARE_ENDSTOPS
    for (uint8_t axis = X_AXIS; axis <= Z_AXIS; ++axis) {
      if (!book_scanner_axis_calibrated[axis]) continue;
      soft_endstop.min[axis] = 0.0f;
      soft_endstop.max[axis] = book_scanner_axis_length[axis];
    }
  #endif
}

bool BookScannerCalibration::load() {
  #if ENABLED(EEPROM_SETTINGS)
    BookScannerCalibrationStore store;
    const int pos = int(persistentStore.capacity() - sizeof(store));

    if (!persistentStore.access_start()) {
      SERIAL_ECHOLNPGM("BookScanner calibration EEPROM unavailable");
      return false;
    }

    const bool error = persistentStore.read_data(pos, (uint8_t *)&store, sizeof(store));
    persistentStore.access_finish();

    if (error || store.magic != BOOK_SCANNER_CALIBRATION_MAGIC || store.version != BOOK_SCANNER_CALIBRATION_VERSION) {
      SERIAL_ECHOLNPGM("BookScanner calibration EEPROM empty");
      return false;
    }

    for (uint8_t axis = X_AXIS; axis <= Z_AXIS; ++axis) {
      const bool calibrated = store.calibrated[axis] != 0 && valid_axis_length(store.length[axis]);
      book_scanner_axis_calibrated[axis] = calibrated;
      book_scanner_axis_length[axis] = calibrated ? store.length[axis] : 0.0f;
    }

    SERIAL_ECHOLNPGM("BookScanner calibration loaded from EEPROM");
    return true;
  #else
    SERIAL_ECHOLNPGM("BookScanner calibration EEPROM disabled");
    return false;
  #endif
}

bool BookScannerCalibration::save() {
  #if ENABLED(EEPROM_SETTINGS)
    BookScannerCalibrationStore store;
    store.magic = BOOK_SCANNER_CALIBRATION_MAGIC;
    store.version = BOOK_SCANNER_CALIBRATION_VERSION;

    for (uint8_t axis = X_AXIS; axis <= Z_AXIS; ++axis) {
      store.calibrated[axis] = book_scanner_axis_calibrated[axis] ? 1 : 0;
      store.length[axis] = book_scanner_axis_length[axis];
    }

    const int pos = int(persistentStore.capacity() - sizeof(store));

    if (!persistentStore.access_start()) {
      SERIAL_ECHOLNPGM("BookScanner calibration EEPROM unavailable");
      return false;
    }

    const bool error = persistentStore.write_data(pos, (const uint8_t *)&store, sizeof(store));
    persistentStore.access_finish();

    SERIAL_ECHOLNPGM(error ? "BookScanner calibration EEPROM save failed" : "BookScanner calibration saved to EEPROM");
    return !error;
  #else
    SERIAL_ECHOLNPGM("BookScanner calibration EEPROM disabled");
    return false;
  #endif
}

#endif // BOOK_SCANNER_CALIBRATION
