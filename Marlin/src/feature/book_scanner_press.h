/**
 * Book scanner press / release motion.
 */

#pragma once

#include "../inc/MarlinConfig.h"

#if ENABLED(BOOK_SCANNER_PRESS)

class BookScannerPress {
public:
  static void clamp();
  static void release();
};

#endif // BOOK_SCANNER_PRESS
