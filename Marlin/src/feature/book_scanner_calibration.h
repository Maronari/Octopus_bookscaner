/**
 * Book scanner axis calibration.
 */

#pragma once

#include "../inc/MarlinConfig.h"

#if ENABLED(BOOK_SCANNER_CALIBRATION)

class BookScannerCalibration {
public:
  static void init();
  static void calibrate(const bool do_x, const bool do_y, const bool do_z);
  static void report();

private:
  static bool calibrate_axis(const AxisEnum axis);
  static void apply_limits();
  static bool load();
  static bool save();
};

#endif // BOOK_SCANNER_CALIBRATION
