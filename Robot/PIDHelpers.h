#ifndef PID_HELPERS
#define PID_HELPERS

#include <Arduino.h>
#include <PID_v1.h>
#include "Utils.h"

namespace PIDHelpers {
  /**
     Reads input from the Serial monitor in the form "kp,ki,kd" and adjusts the PID
     constants of the supplied controller.
  */
  void adjustPIDConstants(PID& controller) {
    Serial.println("Enter your PID constants in the form 'kp,ki,kd': ");

    if (!Utils::awaitUserInput()) {
      Serial.println("Timed out while awaiting user input, keeping PID constants the same.");
      return;
    }

    String user_input = Serial.readString();
  
    // Parse constants kp, ki, kd
    int kp_end = user_input.indexOf(',');
    int ki_end = user_input.indexOf(',', kp_end + 1);
  
    double kp = user_input.substring(0, kp_end).toDouble();
    double ki = user_input.substring(kp_end + 1, ki_end).toDouble();
    double kd = user_input.substring(ki_end + 1).toDouble();
  
    Serial.print("Setting PID constants to: ");
    Serial.print(kp);
    Serial.print(", ");
    Serial.print(ki);
    Serial.print(", ");
    Serial.println(kd);
  
    controller.SetTunings(kp, ki, kd);
  }
};

// TODO: move into namespace
typedef struct PIDConstants_t {
  float kp = 0;
  float ki = 0;
  float kd = 0;
} PIDConstants;

#endif
