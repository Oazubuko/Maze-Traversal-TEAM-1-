#pragma once
#include "LineSensor.h"

// Finite State Machine
enum class State {
  AWAITING_INSTRUCTIONS,
  FOLLOWING_LINE,
  IDENTIFYING_JUNCTION,
  TURNING,
  FINISHED
};

String stateAsString(State state) {
  switch (state) {
    case State::FOLLOWING_LINE: return "Following Line";
    case State::IDENTIFYING_JUNCTION: return "Identifying Junction";
    case State::TURNING: return "Turning";
    case State::FINISHED: return "Finished";
    default: return "Unknown state!";
  }
}

// State actions
void followingLineActions();
void identifyingJunctionActions(LineReading);
void turningActions();
void finishedActions();

// State transitions
State followingLineNextState(LineReading);
State identifyingJunctionNextState();
State turningNextState();
State finishedNextState();
