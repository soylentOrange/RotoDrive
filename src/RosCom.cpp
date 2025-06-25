// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2025 Robert Wendlandt
 */

#include <thingy.h>

#include <functional>

#define TAG "RosCom"

void RosCom::begin(Scheduler* scheduler) {
  // Task handling
  _scheduler = scheduler;
  _allocator = rcl_get_default_allocator();
  _rosComState = RosComState::CONNECTING;

  // Set up a task for initializing the ros-communication
  Task* initROSTask = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _initROS(); }, _scheduler, false, NULL, NULL, true);
  initROSTask->enable();
  initROSTask->waitFor(webSite.getStatusRequest());
}

void RosCom::end() {
  // Stop the spinning task
  if (_spinROSTask != nullptr) {
    _spinROSTask->disable();
    _spinROSTask = nullptr;
  }

  //   // Stop the checking tast task
  //   if (_checkROSTask != nullptr) {
  //     _checkROSTask->disable();
  //     _checkROSTask = nullptr;
  //   }
  _rosComState = RosComState::UNKNOWN;
  _rosEventCallback = nullptr;
}

// Initialization
void RosCom::_initROS() {

  // possibly delay initialization if network isn't connected to WiFi
  // like programming...
  if (eventHandler.getStatusRequest()->pending()) {
    LOGI(TAG, "Delay ROS setup");
    Task* initDelayedROSTask = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _initROS(); }, _scheduler, false, NULL, NULL, true);
    initDelayedROSTask->enable();
    initDelayedROSTask->waitFor(eventHandler.getStatusRequest());
    return;
  }

  if (rclc_support_init(&_support, 0, NULL, &_allocator) != RCL_RET_OK) {
    LOGI(TAG, "rclc_support_init failed: try again later...");
    // Try again later
    Task* initDelayedROSTask = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _initROS(); }, _scheduler, false, NULL, NULL, true);
    initDelayedROSTask->enableDelayed(1000);
    return;
  } else {
    LOGI(TAG, "rclc_support_init succeeded!");
  }

  if (rclc_node_init_default(&_node, "micro_ros_platformio_node", "", &_support) != RCL_RET_OK) {
    LOGI(TAG, "rclc_node_init_default failed: try again later...");
    // Try again later
    Task* initDelayedROSTask = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _initROS(); }, _scheduler, false, NULL, NULL, true);
    initDelayedROSTask->enableDelayed(1000);
    return;
  } else {
    LOGI(TAG, "rclc_node_init_default succeeded!");
  }

  if (rclc_subscription_init_default(&_subscriber, &_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "stepper_pan") != RCL_RET_OK) {
    LOGI(TAG, "rclc_subscription_init_default failed: try again later...");
    // Try again later
    Task* initDelayedROSTask = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _initROS(); }, _scheduler, false, NULL, NULL, true);
    initDelayedROSTask->enableDelayed(1000);
    return;
  } else {
    LOGI(TAG, "rclc_subscription_init_default succeeded!");
  }

  if (rclc_executor_init(&_executor, &_support.context, 1, &_allocator) != RCL_RET_OK) {
    LOGI(TAG, "rclc_executor_init failed: try again later...");
    // Try again later
    Task* initDelayedROSTask = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _initROS(); }, _scheduler, false, NULL, NULL, true);
    initDelayedROSTask->enableDelayed(1000);
    return;
  } else {
    LOGI(TAG, "rclc_executor_init succeeded!");
  }

  if (rclc_executor_add_subscription(&_executor, &_subscriber, &_msg, [](const void* msg_in) { roscom._stepper_command_callback(msg_in); }, ON_NEW_DATA) != RCL_RET_OK) {
    LOGI(TAG, "rclc_executor_add_subscription failed: try again later...");
    // Try again later
    Task* initDelayedROSTask = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _initROS(); }, _scheduler, false, NULL, NULL, true);
    initDelayedROSTask->enableDelayed(1000);
    return;
  } else {
    LOGI(TAG, "rclc_executor_add_subscription succeeded!");
  }

  // Start ROS-Task
  LOGD(TAG, "starting _spinROSTask");
  _spinROSTask = new Task(25, TASK_FOREVER, [&] { _spinROS(); }, _scheduler, false, NULL, NULL, true);
  _spinROSTask->enableDelayed(25);

  _rosComState = RosComState::CONNECTED;

  // execute callback (from website)
  if (_rosEventCallback != nullptr) {
    JsonDocument jsonMsg;
    jsonMsg["type"] = "ros_state";
    jsonMsg["state"] = getRosComState_as_string().c_str();
    jsonMsg.shrinkToFit();
    _rosEventCallback(jsonMsg);
  }

  //   // Start Subscription-check-Task
  //   LOGD(TAG, "starting _checkROSTask");
  //   _checkROSTask = new Task(1000, TASK_FOREVER, [&] { _checkROS(); }, _scheduler, false, NULL, NULL, true);
  //   _checkROSTask->enableDelayed(1000);

  //   // remember starting time
  //   _lastPing = millis();
}

void RosCom::_spinROS() {
  if (rclc_executor_spin_some(&_executor, RCL_MS_TO_NS(10)) == RCL_RET_ERROR) {
    LOGE(TAG, "rclc_executor_spin_some failed");
  }
}

// // check if ros is still alive
// void RosCom::_checkROS() {
//   if ((millis() - _lastPing) > 1500) {
//     LOGW(TAG, "timeout for ros-connection");
//     Task* initDelayedROSTask = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _initROS(); }, _scheduler, false, NULL, NULL, true);
//     initDelayedROSTask->enableDelayed(1000);

//     // Stop the spinning task
//     if (_spinROSTask != nullptr) {
//       _spinROSTask->disable();
//       _spinROSTask = nullptr;
//     }
//   }
// }

void RosCom::_stepper_command_callback(const void* msg_in) {
  auto msg = static_cast<const std_msgs__msg__Int32*>(msg_in);
  // LOGI(TAG, "Received command: %d", msg->data);

  //   // Remember time
  //   _lastPing = millis();

  // only do something when the motor is ready
  if (stepper.getMotorState() == Stepper::MotorState::IDLE || stepper.getMotorState() == Stepper::MotorState::DRIVING) {
    // Only do something when a different command was received
    if (_lastCommand == msg->data) {
      return;
    }

    // Remember last command
    _lastCommand = msg->data;

    switch (_lastCommand) {
      case COMMAND_STOP:
        stepper.halt_move();
        LOGI(TAG, "COMMAND_STOP");
        break;
      case COMMAND_FWD:
        stepper.start_move(36000, HOMING_SPEED, 200);
        LOGI(TAG, "COMMAND_FWD");
        break;
      case COMMAND_REV:
        stepper.start_move(-36000, HOMING_SPEED, 200);
        LOGI(TAG, "COMMAND_REV");
        break;
      case COMMAND_HOME:
        stepper.do_homing();
        LOGI(TAG, "COMMAND_HOME");
        break;
    }
  }
}
