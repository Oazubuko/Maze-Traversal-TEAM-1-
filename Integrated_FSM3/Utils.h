#pragma once

#include <vector>
#include <algorithm>
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

  inline double square(double num) {
    return num * num;
  }

  /**
   * Computes the median value of a given vector in O(n) time. The implementation is
   * "borrowed" from https://stackoverflow.com/questions/1719070/what-is-the-right-approach-when-using-stl-container-for-median-calculation/1719155#1719155
   */
  float median(std::vector<float> &v) {
      size_t midpoint = v.size() / 2;
      nth_element(v.begin(), v.begin() + midpoint, v.end());
      
      return v[midpoint];
  }
};
