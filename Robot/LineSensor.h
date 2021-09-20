/**
 * Wrapper around the two Adafruit_MCP3008 ADCs that hook up to an array of
 * reflectance sensors. Reflectance values are indexed using the labels on the
 * sensor array (e.g., the pin labeled "8" on the Pololu board will be accessed using
 * index 8 in this class). Since the sensor array is 1-based indexed, so is this class.
 */
#ifndef LINE_SENSOR
#define LINE_SENSOR

#include <Adafruit_MCP3008.h>
#include <cstdint>

class LineSensor {
  public:
    constexpr static uint8_t SENSOR_COUNT = 13;

  private:
    constexpr static uint8_t CENTER_RIGHT_PIN = 6;
    constexpr static uint8_t CENTER_PIN = 7;
    constexpr static uint8_t CENTER_LEFT_PIN = 8;

    uint8_t _leftPin;
    uint8_t _rightPin;
    Adafruit_MCP3008 _leftHalf;
    Adafruit_MCP3008 _rightHalf;

    // Reflectance values <= to this will be considered tape
    constexpr static int TAPE_REFLECTANCE_CUTOFF = 700;

  public:
    LineSensor(uint8_t leftPin, uint8_t rightPin) :
      _leftPin(leftPin),
      _rightPin(rightPin),
      _leftHalf(),
      _rightHalf() 
    {}

    
    void begin() {
      _leftHalf.begin(_leftPin);
      _rightHalf.begin(_rightPin);
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

        return -1; // TODO: return exception?
      }

      // The pins are arranged in the order
      // 13 -> 6L
      // 12 -> 5R
      // 11 -> 5L
      // 10 -> 4R
      // ...
      // So we can calculate the pin as follows:
      uint8_t pin = (sensorIndex - 1) / 2;

      // And the direction is "right" for even indices and "left" for odd indices
      if (sensorIndex % 2 == 0) {
        return _rightHalf.readADC(pin);
      }

      return _leftHalf.readADC(pin);
    }
    
    bool isSensorAboveTape(int sensorIndex) {
      return getReflectanceAt(sensorIndex) < TAPE_REFLECTANCE_CUTOFF;
    }

    bool centeredOnTape() {
      return isSensorAboveTape(CENTER_PIN);
    }

    /**
     * Return a skew value that summarizes how much the line sensor is misaligned with the
     * tape below. Skew ranges from [-1, +1]. Positive values indicate that the sensor is
     * too far right of the tape, negative values indicate that the sensor is too far left of the tape.
     */
    float getSkew() {
      int leftSkew = 0;
      int rightSkew = 0;

      for (int i = 1; i <= SENSOR_COUNT; i++) {
        if (i < 7 && isSensorAboveTape(i)) {
          leftSkew++;
        } else if (i > 7 && isSensorAboveTape(i)) {
          rightSkew++;
        }
      }

      float maxSkewPerSide = (SENSOR_COUNT - 1) / 2;

      // Normalize the skew on the range [-1, 1]
      return (rightSkew - leftSkew) / maxSkewPerSide;
    }

    bool cantSeeAnyTape() {
      for (int i = 1; i <= SENSOR_COUNT; i++) {
        if (isSensorAboveTape(i)) return false;
      }

      return true;
    }

    void printAllSensorValues() {
      for (int i = SENSOR_COUNT; i >= 1; i--) {
        Serial.print(isSensorAboveTape(i));
        Serial.print("\t");
      }
      Serial.println();
    }

    // Here is some extra logic to perform sensor calibration. It turns out that it isn't needed
    // since the sensors are more accurate than I thought, but I wanted to leave them in regardless

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

#endif