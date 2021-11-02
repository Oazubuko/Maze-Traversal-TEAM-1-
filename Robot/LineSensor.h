/**
 * Wrapper around the two Adafruit_MCP3008 ADCs that hook up to an array of
 * reflectance sensors. Reflectance values are indexed using the labels on the
 * sensor array (e.g., the pin labeled "8" on the Pololu board will be accessed using
 * index 8 in this class). Since the sensor array is 1-based indexed, so is this class.
 */
#pragma once

#include <Adafruit_MCP3008.h>
#include <cstdint>
#include "Junction.h"

class LineSensor {
  public:
    constexpr static uint8_t SENSOR_COUNT = 13;

  private:
    constexpr static uint8_t CENTER_RIGHT_PIN = 6;
    constexpr static uint8_t CENTER_PIN = 7;
    constexpr static uint8_t CENTER_LEFT_PIN = 8;
    constexpr static int JUNCTION_TAPE_COUNT = 4;
    constexpr static int MAX_TAPE_REFLECTANCE = 690; // Reflectance values <= to this will be considered tape

    uint8_t _leftADCPin;
    uint8_t _rightADCPin;
    Adafruit_MCP3008 _leftADC;
    Adafruit_MCP3008 _rightADC;

  public:
    LineSensor(uint8_t leftADCPin, uint8_t rightADCPin) :
      _leftADCPin(leftADCPin),
      _rightADCPin(rightADCPin),
      _leftADC(),
      _rightADC()
    {}

    /**
     * Attaches the 
     */
    void begin() {
      _leftADC.begin(_leftADCPin);
      _rightADC.begin(_rightADCPin);
    }

    /**
     * Returns the reflectance value from the sensor at the requested index.
     * The index matches the number on the physical board (e.g., request index 8 if you want
     * the sensor labeled as "8" on the Pololu board).
     */
    int getReflectanceAt(int sensorIndex) {
      if (sensorIndex < 1 || sensorIndex > SENSOR_COUNT) {
        Serial.print("Error: '");
        Serial.print(sensorIndex);
        Serial.println("' is not in the range of valid sample indexes (1 - 13 inclusive)");

        return -1;
      }

      // The pins are arranged in the order
      // 13 -> Pin 6 of left ADC
      // 12 -> Pin 5 of right ADC
      // 11 -> Pin 5 of left ADC
      // 10 -> Pin 4 of right ADC
      // ...
      // So we can calculate the pin as follows:
      uint8_t pin = (sensorIndex - 1) / 2;

      // And the ADC is "right" for even indices and "left" for odd indices
      if (sensorIndex % 2 == 0) {
        return _rightADC.readADC(pin);
      }

      return _leftADC.readADC(pin);
    }
    
    bool isSensorAboveTape(int sensorIndex) {
      return getReflectanceAt(sensorIndex) < MAX_TAPE_REFLECTANCE;
    }

    bool centeredOnTape() {
      return isSensorAboveTape(CENTER_PIN);
    }

    /**
     * Returns the number of sensors with tape between the start and end index (inclusive)
     */
    int countTapeBetween(int startIndex, int endIndex) {
      int tapeCount = 0;

      for (int i = startIndex; i <= endIndex; i++) {
        if (isSensorAboveTape(i)) {
          tapeCount++;
        }
      }

      return tapeCount;
    }

    int countRightTape() {
      return countTapeBetween(1, CENTER_PIN - 1);
    }

    int countLeftTape() {
      return countTapeBetween(CENTER_PIN + 1, SENSOR_COUNT);
    }

    /**
     * Return a skew value that summarizes how much the line sensor is misaligned with the
     * tape below. Skew ranges from [-1, +1]. Positive values indicate that the sensor is
     * too far right of the tape, negative values indicate that the sensor is too far left of the tape.
     */
    float getSkew() {
      float maxSkewPerSide = (SENSOR_COUNT - 1) / 2.;

      // Normalize the skew on the range [-1, 1]
      return (countLeftTape() - countRightTape()) / maxSkewPerSide;
    }

    /**
     * An alternate algorithm for calculating skew that gives extra weight to the one
     * Again, skew is a value from [-1, 1]
     */
    float getSkew2() {
      constexpr float MAX_SKEW_PER_SIDE = 21;
      float skew = 0;

      // Values left of the CENTER_PIN contribute positive skew,
      // values to the right of the CENTER pin contribute negative skew
      for(int i = 1; i <= SENSOR_COUNT; i++) {
        if (isSensorAboveTape(i)) {
          skew += i - CENTER_PIN;
        }
      }

      return skew / MAX_SKEW_PER_SIDE;
    }

    bool cantSeeAnyTape() {
      for (int i = 1; i <= SENSOR_COUNT; i++) {
        if (isSensorAboveTape(i)) return false;
      }

      return true;
    }

    void printAllSensorValues() {
      for (int i = SENSOR_COUNT; i >= 1; i--) {
        Serial.print(getReflectanceAt(i));
        Serial.print("\t");
      }
      Serial.println();
    }

    // Looks at the tape 
    Junction identifyJunction() {
      if (cantSeeAnyTape()) {
        return Junction::DEAD_END;
      }

      bool foundLeftJunction = countLeftTape() > JUNCTION_TAPE_COUNT;
      bool foundRightJunction = countRightTape() > JUNCTION_TAPE_COUNT;

      if (foundRightJunction && foundLeftJunction) {
        return Junction::T;
      } else if (foundRightJunction) {
        return Junction::RIGHT;
      } else if (foundLeftJunction) {
        return Junction::LEFT;
      }

      return Junction::LINE;
    }

    // Here is some extra logic to perform sensor calibration. It turns out that it isn't needed
    // since the sensors are more accurate than I thought, but I wanted to leave this code here in case
    // I need it later

    // constexpr static int NUM_CALIBRATION_SAMPLES = 20;
    // int tapeCutoffs[SENSOR_COUNT + 1];
    // /**
    //  * Since each reflectance sensor is different, sample values from each
    //  * sensor to calculate the value below which a sensor reading will be considered white tape.
    //  * For proper calibration, place the robot on a black surface while this function is being called.
    //  */
    // void calibrateTapeCutoffs() {
    //   Serial.println("Calculating tape cutoffs...");

    //   for (int i = 0; i < NUM_CALIBRATION_SAMPLES; i++) {
    //     for (int j = 1; j <= SENSOR_COUNT; j++) {
    //       tapeCutoffs[j] += getReflectanceAt(j);
    //     }
    //     delay(100);
    //   }

    //   // Compute the average reflectance value for each 
    //   for (int i = 1; i <= SENSOR_COUNT; i++) {
    //     tapeCutoffs[i] /= NUM_CALIBRATION_SAMPLES;

    //     // Any value that is below 95% of the average reflectance value counts as tape
    //     tapeCutoffs[i] *= 0.95;
    //   }
    //   Serial.println("Finished calculating tape cutoffs...");
    // }

    // void printTapeCutoffs() {
    //   Serial.println("Light Sensor Cutoffs: ");

    //   for (int i = 1; i <= SENSOR_COUNT; i++) {
    //     Serial.print(i);
    //     Serial.print(": ");
    //     Serial.print(tapeCutoffs[i]);
    //     Serial.print("\t");
    //   }

    //   Serial.println();
    // }
};
