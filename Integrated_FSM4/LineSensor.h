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
#include "Songs.h"

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

/**
 * Summarizes the last few line readings and how confident we are in 
 * the latest reading
 */
typedef struct LineHistory_t {
  LineReading lastConfidentReading = LineReading::LINE;
  LineReading lastReading = LineReading::UNKNOWN;
  unsigned int confidence = 0;
} LineHistory;

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
    LineHistory lineHistory;

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

      // Only run calibration if the tape cutoff reflectance is undefined
#ifndef TAPE_CUTOFF_REFLECTANCE
      calibrate();
#else
      MAX_TAPE_REFLECTANCE = TAPE_CUTOFF_REFLECTANCE;
      Serial.println("Skipping line sensor calibration, using a tape cutoff of " + String(MAX_TAPE_REFLECTANCE));
#endif
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
      Serial.println("Reflectances w/ cutoff (" + String(MAX_TAPE_REFLECTANCE) + "): ");
      String pinLabels;
      String message;

      for (int i = SENSOR_COUNT; i >= 1; i--) {
        message += getReflectanceAt(i);
        message += '\t';

        pinLabels += i;
        pinLabels += '\t';
      }

      Serial.println(pinLabels);
      Serial.println(message);
    }

    /**
       Classifies a reading from the line sensor. This function will only return a new
       line reading after seeing enough "evidence" that we've truly seen that reading.

       For example, if the sensor sees a single LEFT line reading, then many FULL line readings,
       it will never return LineReading::LEFT, only LineReading::FULL after enough evidence.
    */
    LineReading getReading() {
      LineReading newReading = getInstantaneousReading();

      updateLineHistory(newReading);

      return lineHistory.lastConfidentReading;
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

      constexpr int NUM_SAMPLES = 20;

      float blackReflectance = 0;
      float minBlackReflectance = 0;
      float whiteReflectance = 0;

      for (int i = 1; i <= NUM_SAMPLES; i++) {
        // Center pin is above white tape
        whiteReflectance += getReflectanceAt(CENTER_PIN);

        // Non-Center Pins arent above white tape. Because we never want to consider non-tape values
        // as tape, we take the minimum reflectance on the left and right sides.
        float rightReflectance = averageReflectanceBetween(1, CENTER_PIN - 2);
        float leftReflectance = averageReflectanceBetween(CENTER_PIN + 2, SENSOR_COUNT);
        blackReflectance += (leftReflectance + rightReflectance) / 2;

        // Also calculate minimum reflectance of non-tape values
        int rightMin = minReflectanceBetween(1, CENTER_PIN - 2);
        int leftMin = minReflectanceBetween(CENTER_PIN + 2, SENSOR_COUNT);
        minBlackReflectance += min(leftMin, rightMin);

        Serial.println("Average Tape: " + String(whiteReflectance / i));
        Serial.println("Average Not Tape: " + String(blackReflectance / i));
        Serial.println("Min Not Tape: " + String(minBlackReflectance / i));
        printAllSensorValues();

        delay(10);
      }

      // Average all samples
      whiteReflectance /= NUM_SAMPLES;
      blackReflectance /= NUM_SAMPLES;
      minBlackReflectance /= NUM_SAMPLES;

      if (minBlackReflectance <= whiteReflectance) {
        Serial.println("Error: Non Tape Reflectance is lower than than the tape reflectance!");
        Songs::playErrorSong();
      }

      // Set the cutoff at the midpoint between the white and black reflectances.
      // In the worst case, the midpoint is above the below the white reflectance or above
      // the minimum black value, so there's a fallback in those cases
      float idealCutoff = (blackReflectance + whiteReflectance) / 2;
      float realisticCutoff = min(idealCutoff, minBlackReflectance - 5);
      float minPossibleCutoff = whiteReflectance + 5;
      MAX_TAPE_REFLECTANCE = static_cast<int>(max(realisticCutoff, minPossibleCutoff));

      Serial.println("Finished calculating tape cutoff: " + String(MAX_TAPE_REFLECTANCE));
    }

    void printLineHistory() {
      Serial.println("Line History: ");
      
      Serial.print("\tLast Reading: " + String(lineHistory.confidence) + "x "); 
      printLineReading(lineHistory.lastReading);
      
      Serial.print("\tLast Confident Reading: ");
      printLineReading(lineHistory.lastConfidentReading);

      Serial.println();
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
       The average reflectance value between the start and end pin inclusive
    */
    int averageReflectanceBetween(int startPin, int endPin) {
      int totalReflectance = 0;
      int numSamples = endPin - startPin + 1;

      for (int pin = startPin; pin <= endPin; pin++) {
        totalReflectance += getReflectanceAt(pin);
      }

      return totalReflectance / numSamples;
    }

    /**
       The minimum reflectance value between the start and end pin inclusive
    */
    int minReflectanceBetween(int startPin, int endPin) {
      int minReflectance = 2048; // Larger than 1024 max

      for (int pin = startPin; pin <= endPin; pin++) {
        minReflectance = min(minReflectance, getReflectanceAt(pin));
      }

      return minReflectance;
    }

    /**
       Adds a new sensor reading to the line history. If we've observed enough evidence
       of a particular line reading, update our last confident reading.
    */
    void updateLineHistory(LineReading newReading) {
      if (newReading == lineHistory.lastReading) {
        lineHistory.confidence++;
      } else {
        lineHistory.lastReading = newReading;
        lineHistory.confidence = 1;
      }

      if (lineHistory.confidence >= LINE_CONFIDENCE_THRESHOLD) {
        lineHistory.lastConfidentReading = lineHistory.lastReading;
      }
    }

    /**
       Classifies a reading from the line sensor immediately
    */
    LineReading getInstantaneousReading() {
      // Here, we don't use the countLeftTape() and countRightTape() functions
      // because we don't want to include the middle left/right sensors when
      // checking for left/right turns. The middle sensors are regularly during
      // regular line following, which isn't what we want.
      int rightCount = countTapeBetween(1, CENTER_RIGHT_PIN - 1);
      int centerCount = countTapeBetween(CENTER_RIGHT_PIN, CENTER_LEFT_PIN);
      int leftCount = countTapeBetween(CENTER_LEFT_PIN + 1, SENSOR_COUNT);
      int totalCount = leftCount + rightCount + centerCount;

//      Serial.println("Tape counts: (L=" + String(leftCount) + ", C=" + String(centerCount) + ", R=" + String(rightCount) + ")");

      // Determine sensor result based on the location of the tape
      if (totalCount == 0) {
        return LineReading::EMPTY;
      }
      if (centerCount == 3 && (1 <= leftCount && leftCount <= 3) && (1 <= rightCount && rightCount <= 3) ) {
        return LineReading::END_OF_MAZE;
      }
      if (leftCount >= TAPE_COUNT_THRESHOLD && rightCount >= TAPE_COUNT_THRESHOLD) {
        return LineReading::FULL;
      }
      if (leftCount >= TAPE_COUNT_THRESHOLD) {
        return LineReading::LEFT;
      }
      if (rightCount >= TAPE_COUNT_THRESHOLD) {
        return LineReading::RIGHT;
      }

      return LineReading::LINE;

      // TODO: add in UNKNOWN line readings?
    }
};
