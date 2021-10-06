#ifndef UTILS
#define UTILS

#include <Arduino.h>

namespace Utils {
  /**
   * Busy waits for user input at the Serial monitor. Returns true if input was entered,
   * false otherwise.
   */
  bool awaitUserInput(int timeoutMs = 10000) {
    int startTime = millis();
    
    while (!Serial.available()) {
      if (millis() - startTime >= timeoutMs) {
        return false;
      }
      
      delay(50);
    }
  
    return true;
  }
};

#endif
