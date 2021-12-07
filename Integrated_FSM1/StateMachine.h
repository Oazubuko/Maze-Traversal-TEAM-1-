#pragma once
#include "LineSensor.h"

// Finite State Machine
enum class State {
  AWAITING_INSTRUCTIONS,
  FOLLOWING_LINE,
  IDENTIFYING_JUNCTION,
  TURNING,
  FINISHED,
  OPTIMIZED_MAZE_RUN,
  TRANSMITTING_DIRECTIONS,
  SLEEP
};

String stateAsString(State state) {
  switch (state) {
    case State::AWAITING_INSTRUCTIONS: return "Acquiring Instructions From Jetson";
    case State::FOLLOWING_LINE: return "Following Line";
    case State::IDENTIFYING_JUNCTION: return "Identifying Junction";
    case State::TURNING: return "Turning";
    case State::FINISHED: return "Finished";
    case State::OPTIMIZED_MAZE_RUN: return "Run Maze Based on Directions from Jetson";
    case State::TRANSMITTING_DIRECTIONS: return "Transmitting Instructions to Jetson";
    case State::SLEEP: return "Mission Completed. Ceasing Activity ...";
    default: return "Unknown state!";
  }
}

// State actions
void awaitingInstructionsActions();
void followingLineActions();
void identifyingJunctionActions(LineReading);
void turningActions();
void finishedActions();
void optimizedMazeRunActions();
void transmittingDirectionsActions();
void sleepActions();

// State transitions
State awaitingInstructionsNextState();
State followingLineNextState(LineReading);
State identifyingJunctionNextState();
State turningNextState();
State finishedNextState();
State optimizedMazeRunNextState();
State transmittingDirectionsNextState();
State sleepNextState();
