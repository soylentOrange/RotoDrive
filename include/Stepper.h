// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2025 Robert Wendlandt
 */
#pragma once

#include <ArduinoJson.h>
#include <MKSServoCAN.h>
#include <TaskSchedulerDeclarations.h>
#include <TwaiCan.h>

#include <functional>
#include <map>
#include <string>

#define DECREASING false
#define INCREASING true

class Stepper {
  public:
    enum class DriverError {
      UNKNOWN,
      POWER,
      OK,
      TEMPERATURE,
      COIL_A,
      COIL_B
    };

  private:
    std::map<DriverError, std::string> DriverError_string_map = {
      {DriverError::UNKNOWN, "Unknown Error"},
      {DriverError::POWER, "Power Failed"},
      {DriverError::OK, "OK"},
      {DriverError::COIL_A, "Coil A"},
      {DriverError::COIL_B, "Coil B"},
      {DriverError::TEMPERATURE, "Temperature"}};

  public:
    enum class DriverComState {
      UNKNOWN,
      UNINITIALIZED,
      OK,
      ERROR
    };

  private:
    std::map<DriverComState, std::string> DriverComState_string_map = {
      {DriverComState::UNKNOWN, "UNKNOWN"},
      {DriverComState::UNINITIALIZED, "UNINITIALIZED"},
      {DriverComState::OK, "OK"},
      {DriverComState::ERROR, "ERROR"}};

  public:
    enum class MotorState {
      UNKNOWN,
      UNINITIALIZED,
      IDLE,
      HOMING,
      HOMED, // only temporarily
      DRIVING,
      ARRIVED,  // only temporarily
      STOPPED,  // only temporarily
      STOPPING, // only temporarily
      WARNING,  // only temporarily
      ERROR
    };

  private:
    std::map<MotorState, std::string> MotorState_string_map = {
      {MotorState::UNKNOWN, "UNKNOWN"},
      {MotorState::UNINITIALIZED, "UNINITIALIZED"},
      {MotorState::IDLE, "IDLE"},
      {MotorState::HOMING, "HOMING"},
      {MotorState::HOMED, "HOMED"},
      {MotorState::DRIVING, "DRIVING"},
      {MotorState::ARRIVED, "ARRIVED"},
      {MotorState::STOPPED, "STOPPED"},
      {MotorState::STOPPING, "DRIVING"},
      {MotorState::WARNING, "WARNING"},
      {MotorState::ERROR, "ERROR"}};

    // Map MotorState to LEDState
    std::map<MotorState, LED::LEDMode> MotorState_LEDMode_map = {
      {MotorState::UNKNOWN, LED::LEDMode::INITIALIZING},
      {MotorState::UNINITIALIZED, LED::LEDMode::INITIALIZING},
      {MotorState::IDLE, LED::LEDMode::IDLE},
      {MotorState::HOMING, LED::LEDMode::HOMING},
      {MotorState::HOMED, LED::LEDMode::IDLE},
      {MotorState::DRIVING, LED::LEDMode::DRIVING},
      {MotorState::ARRIVED, LED::LEDMode::IDLE},
      {MotorState::STOPPED, LED::LEDMode::IDLE},
      {MotorState::STOPPING, LED::LEDMode::DRIVING},
      {MotorState::WARNING, LED::LEDMode::IDLE},
      {MotorState::ERROR, LED::LEDMode::ERROR}};

    enum class MotorDirection {
      FORWARDS,
      BACKWARDS,
      STANDSTILL
    };

    enum class InitializationState {
      UNITITIALIZED,
      GRADIENT_HOMING,   // Gradient calibration moving towards home
      GRADIENT_HOME,     // Gradient calibration hit home
      GRADIENT_DEHOMING, // Gradient calibration moving away from home
      OK
    };

  public:
    explicit Stepper(TwaiCan& canBus) : _canBus(&canBus) {
      //_srStandstill.setWaiting();
    }
    void begin(Scheduler* scheduler);
    void end();
    typedef std::function<void(JsonDocument doc)> MotorEventCallback;
    void listenMotorEvent(MotorEventCallback callback) { _motorEventCallback = callback; }
    DriverComState getComState() { return _driverComState; }
    std::string getComState_as_string() { return DriverComState_string_map[_driverComState]; }
    MotorState getMotorState() { return _motorState; }
    std::string getMotorState_as_string() { return MotorState_string_map[_motorState]; }
    LED::LEDMode getMotorState_as_LEDMode() { return MotorState_LEDMode_map[_motorState]; }
    void start_move(int32_t position, int32_t speed, int32_t acceleration, int32_t clientID = -1);
    void halt_move();
    void do_homing();
    void set_zero();
    int32_t getCurrentPosition() { return _current_position; }
    int32_t getCurrentSpeed() { return _current_speed; }
    int32_t getDestinationPosition() { return _destination_position; }
    int32_t getDestinationSpeed() { return _destination_speed; }
    int32_t getDestinationAcceleration() { return _destination_acceleration; }
    bool getAutoHome() { return _autoHome; }
    void setAutoHome(bool autoHome);
    std::string getHomingState_as_string();

    // CAN Feedback
    void currentPosition_Feedback(int32_t position);
    void currentSpeed_Feedback(int32_t speed) { _current_speed = abs(speed); }
    void set_zero_Feedback(bool result);
    void move_Feedback(uint8_t result);

  private:
    Scheduler* _scheduler = nullptr;
    TwaiCan* _canBus = nullptr;
    DriverComState _driverComState = DriverComState::UNKNOWN;
    MotorState _motorState = MotorState::UNKNOWN;
    Task* _pollMKSTask = nullptr;
    void _pollMKS();
    void _initMKS();
    InitializationState _initializationState = InitializationState::UNITITIALIZED;
    bool _homed = false;
    bool _autoHome = false;
    // desired position, speed, acceleration in deg, rpm, au
    // (current values will be gathered from FastAccelStepper on demand)
    int32_t _destination_position = 0;
    int32_t _destination_speed = 0;
    int32_t _destination_acceleration = 0;
    // current position, speed in deg, rpm
    int32_t _current_position = 0;
    int32_t _current_speed = 0;
    // MotorDirection _movementDirection = MotorDirection::STANDSTILL;
    Task* _checkMovementTask = nullptr;
    void _checkMovementCallback();
    // void _checkStandstillCallback();
    // StatusRequest _srStandstill;
    // to be called by website for motor specific events
    void _webEventCallback(JsonDocument doc);
    // to be called by stepper for motor specific events
    MotorEventCallback _motorEventCallback = nullptr;
};
