#pragma once
#include "LineSensor.h"



// Finite State Machine
enum class State {
  AWAITING_INSTRUCTIONS,
  FOLLOWING_LINE,
  IDENTIFYING_JUNCTION,
  TURNING,
  FOLLOWING_DIRECTIONS,
  TRANSMITTING_DIRECTIONS,
  FINISHED
};

String stateAsString(State state) {
  switch (state) {
    case State::AWAITING_INSTRUCTIONS: return "Acquiring Instructions From Jetson";
    case State::FOLLOWING_LINE: return "Following Line";
    case State::IDENTIFYING_JUNCTION: return "Identifying Junction";
    case State::TURNING: return "Turning";
    case State::FOLLOWING_DIRECTIONS: return "Following Directions from Jetson";
    case State::TRANSMITTING_DIRECTIONS: return "Transmitting Instructions to Jetson";
    case State::FINISHED: return "Finished";
    default: return "Unknown state!";
  }
}

// State actions
void awaitingInstructionsActions();
void followingLineActions();
void identifyingJunctionActions(LineReading);
void turningActions();
void followingDirectionsActions();
void transmittingDirectionsActions();
void finishedActions();

// State entry functions
void enterFollowingLineState();
void enterIdentifyingJunctionState(LineReading);
void enterTurningState();
void enterFollowingDirectionsState(LineReading);
void enterTransmittingDirectionsState();
void enterFinishedState();
// TODO: add entry functions for integrated states
