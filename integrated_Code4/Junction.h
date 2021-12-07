#pragma once

#include "Arduino.h"

enum class Junction {
  LEFT,
  RIGHT,
  T,
  LINE,
  DEAD_END
};

String junctionAsString(Junction junction) {
  switch (junction)
  {
    case Junction::DEAD_END: return "Dead End";
    case Junction::RIGHT: return "Right";
    case Junction::LEFT: return "Left";
    case Junction::T: return "T";
    case Junction::LINE: return "Line";
    default: return "Unknown junction";
  }
}
