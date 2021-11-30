#pragma once

#include "Arduino.h"

enum class Junction {
  LEFT,
  LEFT_T,
  RIGHT,
  RIGHT_T,
  T,
  PLUS,
  LINE,
  DEAD_END,
  END_OF_MAZE,
  UNKNOWN
};

String junctionAsString(Junction junction) {
  switch (junction)
  {
    case Junction::DEAD_END: return "Dead End";
    case Junction::RIGHT: return "Right (⎾)";
    case Junction::RIGHT_T: return "Right T (⊢)";
    case Junction::LEFT: return "Left (⏋)";
    case Junction::LEFT_T: return "Left T (⊣)";
    case Junction::T: return "T";
    case Junction::PLUS: return "Plus (+)";
    case Junction::LINE: return "Line (|)";
    case Junction::END_OF_MAZE: return "End of Maze (⦀)";
    
    case Junction::UNKNOWN:
    default:
      return "Unknown junction!!";
  }
}
