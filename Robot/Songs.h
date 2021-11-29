#pragma once

#include <Arduino.h>
#include <Buzzer.h>
#include "Constants.h"

static Buzzer buzzer(BUZZER_PIN);

namespace Songs {
  void playSound(int note, unsigned int duration=200) {
    buzzer.sound(note, duration);
  }

  void playErrorSong() {
    buzzer.begin(10);
    
    playSound(NOTE_AS4, 100);
    playSound(NOTE_A4, 100);
    playSound(NOTE_GS4, 100);
    playSound(NOTE_G4, 100);
    playSound(NOTE_FS4, 100);
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
  }

  void playJingleBells() {
    int time = 500;

    buzzer.begin(10);
  
    playSound(NOTE_G3, time / 2);
    playSound(NOTE_E4, time / 2);
    playSound(NOTE_D4, time / 2);
    playSound(NOTE_C4, time / 2);
    playSound(NOTE_G3, time * 2);
    
    playSound(NOTE_G3, time / 2);
    playSound(NOTE_E4, time / 2);
    playSound(NOTE_D4, time / 2);
    playSound(NOTE_C4, time / 2);
    playSound(NOTE_A4, time * 2);
  
    playSound(NOTE_A4, time / 2);
    playSound(NOTE_F4, time / 2);
    playSound(NOTE_E4, time / 2);
    playSound(NOTE_D4, time / 2);
    playSound(NOTE_G4, time);
    playSound(NOTE_G4, time);
    
    playSound(NOTE_A5, time / 2);
    playSound(NOTE_G4, time / 2);
    playSound(NOTE_F4, time / 2);
    playSound(NOTE_D4, time / 2);
    playSound(NOTE_E4, time * 2);
    
    playSound(NOTE_G3, time / 2);
    playSound(NOTE_E4, time / 2);
    playSound(NOTE_D4, time / 2);
    playSound(NOTE_C4, time / 2);
    playSound(NOTE_G3, time * 2);
    
    playSound(NOTE_G3, time / 2);
    playSound(NOTE_E4, time / 2);
    playSound(NOTE_D4, time / 2);
    playSound(NOTE_C4, time / 2);
    playSound(NOTE_A4, time * 2);
    
    playSound(NOTE_A4, time / 2);
    playSound(NOTE_F4, time / 2);
    playSound(NOTE_E4, time / 2);
    playSound(NOTE_D4, time / 2);
    playSound(NOTE_G4, time / 2);
    playSound(NOTE_G4, time / 2);
    playSound(NOTE_G4, (time * 3) / 4);
    playSound(NOTE_G4, time / 4);
    
    playSound(NOTE_A5, time / 2);
    playSound(NOTE_G4, time / 2);
    playSound(NOTE_F4, time / 2);
    playSound(NOTE_D4, time / 2);
    playSound(NOTE_C4, time * 2);
  
    playSound(NOTE_E4, time / 2);
    playSound(NOTE_E4, time / 2);
    playSound(NOTE_E4, time);
    playSound(NOTE_E4, time / 2);
    playSound(NOTE_E4, time / 2);
    playSound(NOTE_E4, time);
    
    playSound(NOTE_E4, time / 2);
    playSound(NOTE_G4, time / 2);
    playSound(NOTE_C4, time / 2);
    playSound(NOTE_D4, time / 2);
    playSound(NOTE_E4, time * 2);
    
    playSound(NOTE_F4, time / 2);
    playSound(NOTE_F4, time / 2);
    playSound(NOTE_F4, time / 2);
    playSound(NOTE_F4, time / 2);
    playSound(NOTE_F4, time / 2);
    playSound(NOTE_E4, time / 2);
    playSound(NOTE_E4, time / 2);
    playSound(NOTE_E4, time / 4);
    playSound(NOTE_E4, time / 4);
  
    playSound(NOTE_E4, time / 2);
    playSound(NOTE_D4, time / 2);
    playSound(NOTE_D4, time / 2);
    playSound(NOTE_E4, time / 2);
    playSound(NOTE_D4, time);
    playSound(NOTE_G4, time);
  
    playSound(NOTE_E4, time / 2);
    playSound(NOTE_E4, time / 2);
    playSound(NOTE_E4, time);
    playSound(NOTE_E4, time / 2);
    playSound(NOTE_E4, time / 2);
    playSound(NOTE_E4, time);
    
    playSound(NOTE_E4, time / 2);
    playSound(NOTE_G4, time / 2);
    playSound(NOTE_C4, time / 2);
    playSound(NOTE_D4, time / 2);
    playSound(NOTE_E4, time * 2);
    
    playSound(NOTE_F4, time / 2);
    playSound(NOTE_F4, time / 2);
    playSound(NOTE_F4, time / 2);
    playSound(NOTE_F4, time / 2);
    playSound(NOTE_F4, time / 2);
    playSound(NOTE_E4, time / 2);
    playSound(NOTE_E4, time / 2);
    playSound(NOTE_E4, time / 4);
    playSound(NOTE_E4, time / 4);
    
    playSound(NOTE_G4, time / 2);
    playSound(NOTE_G4, time / 2);
    playSound(NOTE_F4, time / 2);
    playSound(NOTE_D4, time / 2);
    playSound(NOTE_C4, time * 2);
  }
};
