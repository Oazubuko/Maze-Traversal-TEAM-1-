/**
   Wrapper around the two Adafruit_MCP3008 ADCs that hook up to an array of
   reflectance sensors. Reflectance values are indexed using the labels on the
   sensor array (e.g., the pin labeled "8" on the Pololu board will be accessed using
   index 8 in this class). Since the sensor array is 1-based indexed, so is this class.
*/
#pragma once

#include <Adafruit_MCP3008.h>
#include <cstdint>
#include "Junction.h"

/**
   The tape orientations that the line sensor can detect
*/
enum class LineReading {
  EMPTY,        // No tape seen
  LINE,         // Tape seen on only a few sensors (<= 3)
  FULL,         // Tape seen on almost every sensor (>= 10)
  END_OF_MAZE,  // Tape seen on all but the outermost sensors
  LEFT,         // Tape only seen on the left side
  RIGHT,        // Tape only seen on the right side
  UNKNOWN       // Any other configuration
};

class LineSensor {
  public:
    constexpr static uint8_t SENSOR_COUNT = 13;

  private:
    constexpr static uint8_t CENTER_RIGHT_PIN = 6;
    constexpr static uint8_t CENTER_PIN = 7;
    constexpr static uint8_t CENTER_LEFT_PIN = 8;
    constexpr static int TAPE_COUNT_THRESHOLD = 4;

    // Reflectance values <= to this will be considered tape. Recalibrated each time
    // the sensor is initialized
    int MAX_TAPE_REFLECTANCE = -1;

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
       Initializes the underlying ADCs for the line sensor & calibrates the line sensor.
    */
    void begin() {
      _leftADC.begin(_leftADCPin);
      _rightADC.begin(_rightADCPin);

      calibrate();
    }

    /**
       Returns the reflectance value from the sensor at the requested index.
       The index matches the number on the physical board (e.g., request index 8 if you want
       the sensor labeled as "8" on the Pololu board).
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

    /**
       Return a skew value that summarizes how much the line sensor is misaligned with the
       tape below. Skew ranges from [-1, +1]. Positive values indicate that the sensor is
       too far right of the tape, negative values indicate that the sensor is too far left of the tape.
    */
    float getSkew() {
      float maxSkewPerSide = (SENSOR_COUNT - 1) / 2.;

      // Normalize the skew on the range [-1, 1]
      return (countLeftTape() - countRightTape()) / maxSkewPerSide;
    }

    /**
       An alternate algorithm for calculating skew that gives extra weight to reflectance
       sensors farther away from the center.
       Again, skew is a value from [-1, 1]
    */
    float getSkew2() {
      constexpr float MAX_SKEW_PER_SIDE = 21;
      float skew = 0;

      // Values left of the CENTER_PIN contribute positive skew,
      // values to the right of the CENTER pin contribute negative skew
      for (int i = 1; i <= SENSOR_COUNT; i++) {
        if (isSensorAboveTape(i)) {
          skew += i - CENTER_PIN;
        }
      }

      return skew / MAX_SKEW_PER_SIDE;
    }

    void printAllSensorValues() {
      for (int i = SENSOR_COUNT; i >= 1; i--) {
        Serial.print(getReflectanceAt(i));
        Serial.print("\t");
      }
      Serial.println();
    }

    /**
     * Summarizes a reading from the line sensor
     */
    LineReading getReading() {
      int tapeCount = countTapeBetween(1, SENSOR_COUNT);
      int leftTapeCount = countLeftTape();
      int rightTapeCount = countRightTape();
      
      LineReading reading = LineReading::UNKNOWN;

//      Serial.println("Left, Right, Total: " + String(leftTapeCount) + ", " + String(rightTapeCount) + ", " + String(tapeCount));
//      printAllSensorValues();

      // Determine sensor result based on the location of the tape
      if (tapeCount == 0) {
        reading =  LineReading::EMPTY;
      } else if (1 <= tapeCount && tapeCount <= 3) {
        reading = LineReading::LINE;
      } else if (tapeCount >= 7 && !isSensorAboveTape(1) && !isSensorAboveTape(2) && !isSensorAboveTape(12) && !isSensorAboveTape(13)) {
        reading =  LineReading::END_OF_MAZE;
      } else if (tapeCount >= 10) {
        reading = LineReading::FULL;
      } else if (leftTapeCount >= 4 && rightTapeCount <= 2) {
        reading = LineReading::LEFT;
      } else if (rightTapeCount >= 4 && leftTapeCount <= 2) {
        reading = LineReading::RIGHT;
      }

      return reading;
    }

    static void printLineReading(LineReading lineReading) {
      switch (lineReading)
      {
      case LineReading::EMPTY:
        Serial.println("Empty");
        break;
      case LineReading::LINE:
        Serial.println("Line");
        break;
      case LineReading::END_OF_MAZE:
        Serial.println("End of Maze");
        break;
      case LineReading::FULL:
        Serial.println("Full");
        break;
      case LineReading::LEFT:
        Serial.println("Left");
        break;
      case LineReading::RIGHT:
        Serial.println("Right");
        break;
      case LineReading::UNKNOWN:      
      default:
        Serial.println("Unknown");
      }
    }

  private:
    /**
       Returns the number of sensors with tape between the start and end index (inclusive)
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
      return countTapeBetween(1, CENTER_RIGHT_PIN);
    }

    int countLeftTape() {
      return countTapeBetween(CENTER_LEFT_PIN, SENSOR_COUNT);
    }

    /**
       Determine the value below which a sensor reading will be considered white tape.

       This is needed to handle different levels of ambient light.

       For proper calibration, place the center reflectance sensor above tape while this 
       function is being called (see the diagram below):
                        |_______
                        |       |
              ===TAPE===| ROBOT |======
                        |_______|
                        |
    */
    void calibrate() {      
      Serial.println("Calculating tape cutoff threshold...");

      constexpr int NUM_SAMPLES = 100;

      float nonTapeReflectance = 0;
      float tapeReflectance = 0;

      for (int i = 1; i <= NUM_SAMPLES; i++) {
        // Center pin is above white tape
        tapeReflectance += getReflectanceAt(CENTER_PIN);

        // Non-Center Pins arent above white tape
        float rightReflectance = averageReflectanceBetween(1, CENTER_PIN - 2);
        float leftReflectance = averageReflectanceBetween(CENTER_PIN + 2, SENSOR_COUNT);
        nonTapeReflectance += (leftReflectance + rightReflectance) / 2;

        Serial.println("Average Tape: " + String(tapeReflectance / i));
        Serial.println("Average Not Tape: " + String(nonTapeReflectance / i));

        delay(10);
      }

      // Set the cutoff threshold at the midpoint of the reflectance values
      float middleReflectance = (tapeReflectance + nonTapeReflectance) / NUM_SAMPLES / 2;
      MAX_TAPE_REFLECTANCE = static_cast<int>(middleReflectance);

      Serial.println("Finished calculating tape cutoff: " + String(MAX_TAPE_REFLECTANCE));
    }

    /**
       The average reflectance value between the start and end pin inclusive
    */
    float averageReflectanceBetween(int startPin, int endPin) {
      float totalReflectance = 0;
      int numSamples = endPin - startPin + 1;

      for (int pin = startPin; pin <= endPin; pin++) {
        totalReflectance += getReflectanceAt(pin);
      }

      return totalReflectance / numSamples;
    }
};
