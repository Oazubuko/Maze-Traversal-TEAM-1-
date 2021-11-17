#pragma once

#include <Arduino.h>
#include "Songs.h"

/**
 * Enter an error state until the end of time! Spooky.
 */
void handleFatalError(String errorMessage) {
  while (true) {
    Serial.println("Encountered a fatal error: " + errorMessage);
    Songs::playMarioTheme();
    delay(1000);
  }
}
