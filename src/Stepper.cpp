// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2025 Robert Wendlandt
 */

#include <FunctionalInterrupt.h>
#include <Preferences.h>
#include <thingy.h>

#include <functional>

#define TAG "Stepper"

void Stepper::begin(Scheduler* scheduler) {
  // Task handling
  _scheduler = scheduler;

  // register listener to website
  LOGD(TAG, "register event handler to website");
  webSite.listenWebEvent([&](JsonDocument doc) { _webEventCallback(doc); });

  // handle persistent options (auto homing...)
  LOGD(TAG, "Get persistent options from preferences...");
  Preferences preferences;
  preferences.begin("rdrive", true);
  _destination_speed = preferences.getInt("speed", 30);
  _destination_acceleration = preferences.getInt("acc", 0);
  _autoHome = preferences.getBool("ahome", false);
  _position_offset = preferences.getInt("offs", 0);
  preferences.end();

  // Set up a task for initializing the motor
  //_initializationState = InitializationState::UNITITIALIZED;
  _driverComState = DriverComState::UNKNOWN;
  _motorState = MotorState::UNKNOWN;
  Task* initMKSTask = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _initMKS(); }, _scheduler, false, NULL, NULL, true);
  initMKSTask->enable();
  initMKSTask->waitFor(webSite.getStatusRequest());
}

void Stepper::end() {
  _motorEventCallback = nullptr;
  //_initializationState = InitializationState::UNITITIALIZED;

  // end the check-task
  if (_pollMKSTask != nullptr) {
    _pollMKSTask->disable();
    _pollMKSTask = nullptr;
  }

  // possibly stop an ongoing movement
  // TODO

  // software-disable the driver
  MKSServoCAN::enableMotor(CAN_ID, false);
}

void Stepper::setAutoHome(bool autoHome) {
  LOGI(TAG, "AutoHoming: %s", autoHome ? "On" : "Off");
  // save if value differs from known
  if (_autoHome != autoHome) {
    _autoHome = autoHome;
    Preferences preferences;
    preferences.begin("rdrive", false);
    preferences.putBool("ahome", _autoHome);
    preferences.end();
  }
}

// Initialization
void Stepper::_initMKS() {
  // possibly delay initialization if network isn't connected to WiFi
  // like programming...
  if (eventHandler.getStatusRequest()->pending()) {
    LOGI(TAG, "Delay MKS setup");
    Task* initDelayedTMC2209Task = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _initMKS(); }, _scheduler, false, NULL, NULL, true);
    initDelayedTMC2209Task->enable();
    initDelayedTMC2209Task->waitFor(eventHandler.getStatusRequest());
    return;
  }

  // Initalization success
  bool initError = false;

  // Reflect state
  _driverComState = DriverComState::UNKNOWN;
  _motorState = MotorState::UNINITIALIZED;
  //_initializationState = InitializationState::UNITITIALIZED;
  led.setMode(LED::LEDMode::INITIALIZING);

  // Start communication with MKS
  if (!MKSServoCAN::begin(_canBus)) { // Communication with CAN failed
    LOGW(TAG, "Driver is not communicating, delay initialization");
    initError = true;
  }

  // CAN init failed
  if (initError) {
    _driverComState = DriverComState::ERROR;
    led.setMode(LED::LEDMode::ERROR);

    // execute callback (from website)
    if (_motorEventCallback != nullptr) {
      JsonDocument jsonMsg;
      jsonMsg["type"] = "motor_state";
      jsonMsg["state"] = getMotorState_as_string().c_str();
      jsonMsg.shrinkToFit();
      _motorEventCallback(jsonMsg);
    }

    // delay initialization if driver is not communicating, yet
    Task* initDelayedStartupMKSTask = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _initMKS(); }, _scheduler, false, NULL, NULL, true);
    initDelayedStartupMKSTask->enableDelayed(1000);
    return;
  }

  // Set up a task for continuously monitoring the driver
  if (_pollMKSTask == nullptr) {
    LOGD(TAG, "starting _pollMKSTask");
    _pollMKSTask = new Task(25, TASK_FOREVER, [&] { _pollMKS(); }, _scheduler, false, NULL, NULL, true);
    _pollMKSTask->enableDelayed(25);
  }

  // CAN init successful...
  LOGI(TAG, "Stepper driver seems fine!");
  //_initializationState = InitializationState::OK;
  _driverComState = DriverComState::OK;
  _motorState = MotorState::IDLE;
  led.setMode(LED::LEDMode::IDLE);

  // Enable the motor
  MKSServoCAN::enableMotor(CAN_ID, true);

  // Read current position
  MKSServoCAN::readEncoderAdd(CAN_ID);

  // possibly do power-on homing
  if (!_homed && _autoHome) {
    do_homing();
  } else {
    // execute callback (from website)
    if (_motorEventCallback != nullptr) {
      JsonDocument jsonMsg;
      jsonMsg["type"] = "motor_state";
      jsonMsg["state"] = getMotorState_as_string().c_str();
      jsonMsg["move_state"]["position"] = 0;
      jsonMsg["move_state"]["speed"] = 0;
      jsonMsg.shrinkToFit();
      _motorEventCallback(jsonMsg);
    }
  }
}

// Check for possible responses from MKS
void Stepper::_pollMKS() {
  // Do it twice (e.g. for position and speed)
  MKSServoCAN::pollResponses(1);
  MKSServoCAN::pollResponses(1);
}

void Stepper::_webEventCallback(JsonDocument doc) {
  LOGD(TAG, "Received Command: %s from client: %d", doc["type"].as<const char*>(), doc["origin"].as<int32_t>());

  // Move command
  if (strcmp(doc["type"].as<const char*>(), "move") == 0) {
    LOGD(TAG, "Motor shall move to %d deg at %d rpm with %d (acc)", doc["position"].as<int32_t>(), doc["speed"].as<int32_t>(), doc["acceleration"].as<int32_t>());

    // Can we start/update a movement?
    if ((_motorState == MotorState::DRIVING) || (_motorState == MotorState::IDLE)) {
      if (_destination_position == doc["position"].as<int32_t>() && _destination_speed == doc["speed"].as<int32_t>()) {
        LOGD(TAG, "Motor movement parameters are identical to current move!");
        // send websock event
        if (_motorEventCallback != nullptr) {
          JsonDocument jsonMsg;
          jsonMsg["type"] = "motor_state";
          jsonMsg["state"] = MotorState_string_map[MotorState::ARRIVED].c_str();
          jsonMsg.shrinkToFit();
          _motorEventCallback(jsonMsg);
        }
        return;
      }

      if (doc["speed"].as<int32_t>() == 0) {
        LOGD(TAG, "Motor speed is 0!");
        // send websock event
        if (_motorEventCallback != nullptr) {
          JsonDocument jsonMsg;
          jsonMsg["type"] = "motor_state";
          jsonMsg["state"] = MotorState_string_map[MotorState::WARNING].c_str();
          jsonMsg["warning"] = "Speed unplausible!";
          jsonMsg.shrinkToFit();
          _motorEventCallback(jsonMsg);
        }
        return;
      }
    } else {
      LOGW(TAG, "Motor movement not allowed! (State: %s)", MotorState_string_map[_motorState].c_str());
      // send websock event
      if (_motorEventCallback != nullptr) {
        JsonDocument jsonMsg;
        jsonMsg["type"] = "motor_state";
        jsonMsg["state"] = MotorState_string_map[MotorState::WARNING].c_str();
        jsonMsg["warning"] = "Movement not allowed!";
        jsonMsg.shrinkToFit();
        _motorEventCallback(jsonMsg);
      }
      return;
    }

    start_move(doc["position"].as<int32_t>(), doc["speed"].as<int32_t>(), doc["acceleration"].as<int32_t>(), doc["origin"].as<int32_t>());
  } else if (strcmp(doc["type"].as<const char*>(), "stop") == 0) { // Stop command
    LOGD(TAG, "Motor shall be stopped");

    // Can we stop a movement?
    if ((_motorState == MotorState::DRIVING) || (_motorState == MotorState::HOMING)) {
      halt_move();
    } else {
      LOGW(TAG, "Stopping not allowed!");
      // send websock event
      if (_motorEventCallback != nullptr) {
        JsonDocument jsonMsg;
        jsonMsg["type"] = "motor_state";
        jsonMsg["state"] = MotorState_string_map[MotorState::WARNING].c_str();
        jsonMsg["warning"] = "Stopping not allowed!";
        jsonMsg.shrinkToFit();
        _motorEventCallback(jsonMsg);
      }
    }
  } else if (strcmp(doc["type"].as<const char*>(), "returnZero") == 0) { // Homing command
    LOGD(TAG, "Motor shall return to Zero");

    // Can we start the homing procedure?
    if (_motorState == MotorState::IDLE) {
      do_homing();
    } else {
      LOGW(TAG, "Return to Zero not allowed!");
      // send websock event
      if (_motorEventCallback != nullptr) {
        JsonDocument jsonMsg;
        jsonMsg["type"] = "motor_state";
        jsonMsg["state"] = MotorState_string_map[MotorState::WARNING].c_str();
        jsonMsg["warning"] = "Return to Zero not allowed!";
        jsonMsg.shrinkToFit();
        _motorEventCallback(jsonMsg);
      }
    }
  } else if (strcmp(doc["type"].as<const char*>(), "setZero") == 0) { // set Zero command
    LOGD(TAG, "Current position set to zero");

    // Can we start the procedure?
    if (_motorState == MotorState::IDLE) {
      set_zero();
    } else {
      LOGW(TAG, "Set Zero not allowed!");
      // send websock event
      if (_motorEventCallback != nullptr) {
        JsonDocument jsonMsg;
        jsonMsg["type"] = "motor_state";
        jsonMsg["state"] = MotorState_string_map[MotorState::WARNING].c_str();
        jsonMsg["warning"] = "Set Zero not allowed!";
        jsonMsg.shrinkToFit();
        _motorEventCallback(jsonMsg);
      }
    }
  } else if (strcmp(doc["type"].as<const char*>(), "update_config") == 0) { // Config command
    LOGD(TAG, "Update config");

    // Update config
    stepper.setAutoHome(doc["autoHome"].as<bool>());

    // send websock event
    if (_motorEventCallback != nullptr) {
      JsonDocument jsonMsg;
      jsonMsg["type"] = "config";
      jsonMsg["autoHome"] = stepper.getAutoHome();
      jsonMsg["origin"] = doc["origin"].as<int32_t>();
      jsonMsg.shrinkToFit();
      _motorEventCallback(jsonMsg);
    }
  } else {
    // send websock event
    if (_motorEventCallback != nullptr) {
      JsonDocument jsonMsg;
      jsonMsg["type"] = "motor_state";
      jsonMsg["state"] = MotorState_string_map[MotorState::WARNING].c_str();
      jsonMsg["warning"] = "Unknown command received!";
      jsonMsg.shrinkToFit();
      _motorEventCallback(jsonMsg);
    }
  }
}

void Stepper::start_move(int32_t position, int32_t speed, int32_t acceleration, int32_t clientID) {
  LOGD(TAG, "Motor will move!");

  // save speed and/or acceleration if values differ from known
  if (_destination_speed != speed || _destination_acceleration != acceleration) {
    Preferences preferences;
    preferences.begin("rdrive", false);
    if (_destination_speed != speed) {
      preferences.putInt("speed", speed);
    }
    if (_destination_acceleration != acceleration) {
      preferences.putInt("acc", acceleration);
    }
    preferences.end();
  }

  // store parameters
  _destination_position = position;
  _destination_speed = speed;
  _destination_acceleration = acceleration;
  float encPosition = (position + _position_offset) * 16384.0 / 360.0;

  if (_motorState != MotorState::DRIVING) {
    _motorState = MotorState::DRIVING;
    led.setMode(LED::LEDMode::DRIVING);

    // update position and speed regularly
    _checkMovementTask = new Task(MOVEMENT_UPDATE_MS, TASK_FOREVER, [&] { _checkMovementCallback(); }, _scheduler, false, NULL, NULL, true);
    _checkMovementTask->enableDelayed(MOVEMENT_UPDATE_MS);
  }

  // send websock event
  if (_motorEventCallback != nullptr) {
    JsonDocument jsonMsg;
    jsonMsg["type"] = "motor_state";
    jsonMsg["origin"] = clientID;
    jsonMsg["state"] = stepper.getMotorState_as_string().c_str();
    jsonMsg["destination"]["position"] = _destination_position;
    jsonMsg["destination"]["speed"] = _destination_speed;
    jsonMsg["destination"]["acceleration"] = _destination_acceleration;
    jsonMsg.shrinkToFit();
    _motorEventCallback(jsonMsg);
  }

  // send command to MKS
  MKSServoCAN::posAxisAbsolute(CAN_ID, static_cast<int16_t>(encPosition), static_cast<uint16_t>(speed), static_cast<uint8_t>(_destination_acceleration));

  // Get current position and speed
  MKSServoCAN::readEncoderAdd(CAN_ID);
  MKSServoCAN::readSpeed(CAN_ID);
}

void Stepper::halt_move() {
  LOGD(TAG, "Motor will stop!");

  // Forcefully stop driving operation
  if (_motorState == MotorState::DRIVING) {
    LOGD(TAG, "Driving Cancelled!");

    // movement is finished anyways
    if (_checkMovementTask != nullptr) {
      _checkMovementTask->disable();
      _checkMovementTask = nullptr;
    }

    // Handle Feedback later (in move_Feddback)...
    _destination_speed = 0;
    _motorState = MotorState::STOPPING;

    // Send stop command
    MKSServoCAN::posAxisAbsolute(CAN_ID, 0, 0, static_cast<uint8_t>(_destination_acceleration));
  } else {
    LOGD(TAG, "Movement Cancelled!");
    _motorState = MotorState::IDLE;
    led.setMode(LED::LEDMode::IDLE);
    _destination_position = _current_position;
    _destination_speed = 0;

    // send websock event
    if (_motorEventCallback != nullptr) {
      JsonDocument jsonMsg;
      jsonMsg["type"] = "motor_state";
      jsonMsg["state"] = MotorState_string_map[MotorState::STOPPED].c_str();
      jsonMsg["move_state"]["position"] = _destination_position;
      jsonMsg["move_state"]["speed"] = 0;
      jsonMsg.shrinkToFit();
      _motorEventCallback(jsonMsg);
    }
  }
}

// Axis Absolute Position Feedback
void Stepper::move_Feedback(uint8_t result) {
  // operation started, don't care
  if (result == 1) {
    return;
  }

  // LOGD(TAG, "move_Feedback result: %d (%s)", result, getMotorState_as_string().c_str());

  // currently going to zero
  if (_motorState == MotorState::HOMING) {
    LOGD(TAG, "Homing finished");
    set_zero_Feedback(result == 2);
  } else if (_motorState == MotorState::DRIVING) {
    LOGD(TAG, "Driving finished");
    // movement is finished anyways
    if (_checkMovementTask != nullptr) {
      _checkMovementTask->disable();
      _checkMovementTask = nullptr;
    }

    // arrived at destination
    if (result == 2) {
      _motorState = MotorState::IDLE;
      led.setMode(LED::LEDMode::IDLE);

      // send websock event
      if (_motorEventCallback != nullptr) {
        JsonDocument jsonMsg;
        jsonMsg["type"] = "motor_state";
        jsonMsg["state"] = MotorState_string_map[MotorState::STOPPED].c_str();
        jsonMsg["move_state"]["position"] = _destination_position;
        jsonMsg["move_state"]["speed"] = 0;
        jsonMsg.shrinkToFit();
        _motorEventCallback(jsonMsg);
      }
    }
  } else if (_motorState == MotorState::STOPPING) {
    LOGD(TAG, "STOPPING State in move feedback");
    _motorState = MotorState::STOPPED;

    // Get current position
    MKSServoCAN::readEncoderAdd(CAN_ID);

    // handle the position update later (in )...

  } else { // Some error has occurred
    LOGD(TAG, "ERROR State in move feedback");
    _motorState = MotorState::ERROR;
    led.setMode(LED::LEDMode::ERROR);

    // Get current position
    MKSServoCAN::readEncoderAdd(CAN_ID);

    // handle the position update later...
  }
}

void Stepper::currentPosition_Feedback(int32_t position) {
  _current_position = position - _position_offset;

  if (_motorState == MotorState::ERROR) {
    // movement is already finished
    if (_checkMovementTask != nullptr) {
      _checkMovementTask->disable();
      _checkMovementTask = nullptr;
    }

    // send websock event
    if (_motorEventCallback != nullptr) {
      JsonDocument jsonMsg;
      jsonMsg["type"] = "motor_state";
      jsonMsg["state"] = MotorState_string_map[MotorState::ERROR].c_str();
      jsonMsg["move_state"]["position"] = _current_position;
      jsonMsg["move_state"]["speed"] = 0;
      jsonMsg.shrinkToFit();
      _motorEventCallback(jsonMsg);
    }
  } else if (_motorState == MotorState::STOPPED) {
    _destination_position = _current_position;
    _motorState = MotorState::IDLE;
    led.setMode(LED::LEDMode::IDLE);

    // send websock event
    if (_motorEventCallback != nullptr) {
      JsonDocument jsonMsg;
      jsonMsg["type"] = "motor_state";
      jsonMsg["state"] = MotorState_string_map[MotorState::STOPPED].c_str();
      jsonMsg["move_state"]["position"] = _current_position;
      jsonMsg["move_state"]["speed"] = 0;
      jsonMsg.shrinkToFit();
      _motorEventCallback(jsonMsg);
    }
  } else if (_motorState == MotorState::ZEROING) {
    LOGI(TAG, "Current position is now zero!");
    _destination_position = 0;
    _position_offset = _current_position;
    _current_position = 0;
    _current_speed = 0;
    _destination_position = 0;
    _homed = true;
    _motorState = MotorState::IDLE;
    led.setMode(LED::LEDMode::IDLE);

    // save current position
    Preferences preferences;
    preferences.begin("rdrive", false);
    preferences.putInt("offs", _position_offset);
    preferences.end();

    // send websock event
    if (_motorEventCallback != nullptr) {
      JsonDocument jsonMsg;
      jsonMsg["type"] = "motor_state";
      jsonMsg["state"] = MotorState_string_map[MotorState::HOMED].c_str();
      jsonMsg["move_state"]["position"] = 0;
      jsonMsg["move_state"]["speed"] = 0;
      jsonMsg.shrinkToFit();
      _motorEventCallback(jsonMsg);
    }
  }
}

void Stepper::do_homing() {
  LOGD(TAG, "Motor will return to zero!");

  _motorState = MotorState::HOMING;
  led.setMode(LED::LEDMode::HOMING);

  // send websock event
  if (_motorEventCallback != nullptr) {
    JsonDocument jsonMsg;
    jsonMsg["type"] = "motor_state";
    jsonMsg["state"] = stepper.getMotorState_as_string().c_str();
    jsonMsg["move_state"]["position"] = 0;
    jsonMsg["move_state"]["speed"] = 50;
    jsonMsg.shrinkToFit();
    _motorEventCallback(jsonMsg);
  }

  // Issue Command to MKS
  float encPosition = _position_offset * 16384.0 / 360.0;
  MKSServoCAN::posAxisAbsolute(CAN_ID, static_cast<int16_t>(encPosition), HOMING_SPEED, HOMING_ACCELERATION);
}

void Stepper::set_zero_Feedback(bool result) {
  LOGI(TAG, "Current position is now zero!");

  // Set internal position to 0
  _current_position = 0;
  _current_speed = 0;
  _destination_position = 0;

  if (result) {
    _homed = true;
    _motorState = MotorState::IDLE;
    led.setMode(LED::LEDMode::IDLE);
  } else {
    _homed = false;
    _motorState = MotorState::ERROR;
    led.setMode(LED::LEDMode::ERROR);
  }

  // send websock event
  if (_motorEventCallback != nullptr) {
    JsonDocument jsonMsg;
    jsonMsg["type"] = "motor_state";
    if (_homed) {
      jsonMsg["state"] = MotorState_string_map[MotorState::HOMED].c_str();
    } else {
      jsonMsg["state"] = MotorState_string_map[MotorState::ERROR].c_str();
    }
    jsonMsg["move_state"]["position"] = 0;
    jsonMsg["move_state"]["speed"] = 0;
    jsonMsg.shrinkToFit();
    _motorEventCallback(jsonMsg);
  }
}

void Stepper::set_zero() {
  LOGD(TAG, "Setting current position to Zero!");

  _motorState = MotorState::ZEROING;
  led.setMode(LED::LEDMode::HOMING);
  _position_offset = 0;
  _homed = false;

  // send websock event
  if (_motorEventCallback != nullptr) {
    JsonDocument jsonMsg;
    jsonMsg["type"] = "motor_state";
    jsonMsg["state"] = stepper.getMotorState_as_string().c_str();
    jsonMsg["move_state"]["position"] = 0;
    jsonMsg["move_state"]["speed"] = 0;
    jsonMsg.shrinkToFit();
    _motorEventCallback(jsonMsg);
  }

  // Send command to MKS
  // MKSServoCAN::setZeroPoint(CAN_ID);

  // Get current position
  MKSServoCAN::readEncoderAdd(CAN_ID);
}

std::string Stepper::getHomingState_as_string() {
  if (_homed) {
    return std::string("OK");
  } else if (_motorState == MotorState::HOMING) {
    return std::string("HOMING");
  } else {
    return std::string("UNHOMED");
  }
}

void Stepper::_checkMovementCallback() {
  // Get current position
  MKSServoCAN::readEncoderAdd(CAN_ID);

  // Get current speed
  MKSServoCAN::readSpeed(CAN_ID);

  // send websock event (will show the last known position and speed though)
  if (_motorEventCallback != nullptr) {
    JsonDocument jsonMsg;
    jsonMsg["type"] = "move_state";
    jsonMsg["position"] = _current_position;
    jsonMsg["speed"] = _current_speed;
    jsonMsg.shrinkToFit();
    _motorEventCallback(jsonMsg);
  }
}
