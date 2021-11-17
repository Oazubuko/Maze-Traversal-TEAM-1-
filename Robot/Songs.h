#pragma once

#include <Arduino.h>
#include <Buzzer.h>
#include "Constants.h"

static Buzzer buzzer(BUZZER_PIN);

namespace Songs {
  void playSound(int note, unsigned int duration) {
    buzzer.sound(note, duration);
  }

  void playStarWarsTheme() {
    buzzer.begin(10);
    
    playSound(NOTE_A3, 500);
    playSound(NOTE_A3, 500);
    playSound(NOTE_A3, 500);
    playSound(NOTE_F3, 375);
    playSound(NOTE_C4, 125);

    playSound(NOTE_A3, 500);
    playSound(NOTE_F3, 375);
    playSound(NOTE_C4, 125);
    playSound(NOTE_A3, 1000);

    playSound(NOTE_E4, 500); 
    playSound(NOTE_E4, 500);
    playSound(NOTE_E4, 500);
    playSound(NOTE_F4, 375);
    playSound(NOTE_C4, 125);

    playSound(NOTE_GS3, 500);
    playSound(NOTE_F3, 375);
    playSound(NOTE_C4, 125);
    playSound(NOTE_A3, 1000);
    
    buzzer.end(100);
  }

  void playMarioTheme() {
    buzzer.begin(100);
    
    playSound(NOTE_E7, 80);
    playSound(NOTE_E7, 80);
    playSound(0, 80);
    playSound(NOTE_E7, 80);
    playSound(0, 80);
    playSound(NOTE_C7, 80);
    playSound(NOTE_E7, 80);
    playSound(0, 80);
    playSound(NOTE_G7, 80);
    playSound(0, 240);
    playSound(NOTE_G6, 80);
    playSound(0, 240);
    
    buzzer.end(100);
  }
};
