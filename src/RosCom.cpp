// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2025 Robert Wendlandt
 */

#include <thingy.h>

#include <functional>

#define TAG "RosCom"

#define RCNOCHECK(fn)       \
  {                         \
    rcl_ret_t temp_rc = fn; \
    (void)temp_rc;          \
  }

void RosCom::begin(Scheduler* scheduler) {
  // Task handling
  _scheduler = scheduler;
  _allocator = rcl_get_default_allocator();
  _rosComState = RosComState::WAITING_AGENT;

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

  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&_support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  RCNOCHECK(rclc_executor_fini(&_executor));
  RCNOCHECK(rcl_subscription_fini(&_subscriber, &_node));
  RCNOCHECK(rcl_node_fini(&_node));
  RCNOCHECK(rclc_support_fini(&_support));

  _rosComState = RosComState::AGENT_DISCONNECTED;
  _rosEventCallback = nullptr;
}

// Initialization
void RosCom::_initROS() {
  // end the Task when it's already here
  if (_spinROSTask != nullptr) {
    _spinROSTask->disable();
    _spinROSTask = nullptr;
  }

  // possibly delay initialization if network isn't connected to WiFi
  // like programming...
  if (eventHandler.getStatusRequest()->pending()) {
    LOGI(TAG, "WiFi not ready - Delay ROS setup");
    Task* initDelayedROSTask = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _initROS(); }, _scheduler, false, NULL, NULL, true);
    initDelayedROSTask->enable();
    initDelayedROSTask->waitFor(eventHandler.getStatusRequest());
    return;
  }

  // possibly delay initialization if agent is not pingable
  if (RMW_RET_OK != rmw_uros_ping_agent(1, 10)) {
    LOGI(TAG, "Agent not available - Delay ROS setup");
    _rosComState = RosComState::AGENT_DISCONNECTED;
    Task* initDelayedROSTask = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _initROS(); }, _scheduler, false, NULL, NULL, true);
    initDelayedROSTask->enableDelayed(1000);
    return;
  } else {
    LOGI(TAG, "Agent is available - proceed ROS setup");
    _rosComState = RosComState::WAITING_AGENT;
  }

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  if (rcl_init_options_init(&init_options, _allocator) != RCL_RET_OK) {
    LOGI(TAG, "rcl_init_options_init failed: try again later...");
    rcutils_reset_error();
    Task* initDelayedROSTask = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _initROS(); }, _scheduler, false, NULL, NULL, true);
    initDelayedROSTask->enableDelayed(1000);
    return;
  }

  // set DOMAIN_ID
  if (rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID) != RCL_RET_OK) {
    LOGI(TAG, "rcl_init_options_set_domain_id failed: try again later...");
    rcutils_reset_error();
    Task* initDelayedROSTask = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _initROS(); }, _scheduler, false, NULL, NULL, true);
    initDelayedROSTask->enableDelayed(1000);
    return;
  }

  // Setup support structure.
  if (rclc_support_init_with_options(&_support, 0, NULL, &init_options, &_allocator) != RCL_RET_OK) {
    LOGI(TAG, "rcl_init_options_set_domain_id failed: try again later...");
    rcutils_reset_error();
    Task* initDelayedROSTask = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _initROS(); }, _scheduler, false, NULL, NULL, true);
    initDelayedROSTask->enableDelayed(1000);
    return;
  }

  // Clean up initialization options
  if (rcl_init_options_fini(&init_options) != RCL_RET_OK) {
    LOGI(TAG, "rcl_init_options_fini failed: try again later...");
    rcutils_reset_error();
    rmw_context_t* rmw_context = rcl_context_get_rmw_context(&_support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    rclc_support_fini(&_support);
    // Try again later
    Task* initDelayedROSTask = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _initROS(); }, _scheduler, false, NULL, NULL, true);
    initDelayedROSTask->enableDelayed(1000);
    return;
  }

  // Create Node
  if (rclc_node_init_default(&_node, "rdrive_node", "", &_support) != RCL_RET_OK) {
    LOGI(TAG, "rclc_node_init_default failed: try again later...");
    rcutils_reset_error();
    rmw_context_t* rmw_context = rcl_context_get_rmw_context(&_support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    RCNOCHECK(rcl_node_fini(&_node));
    RCNOCHECK(rclc_support_fini(&_support));
    // Try again later
    Task* initDelayedROSTask = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _initROS(); }, _scheduler, false, NULL, NULL, true);
    initDelayedROSTask->enableDelayed(1000);
    return;
  } else {
    LOGI(TAG, "rclc_node_init_default succeeded!");
  }

  if (rclc_subscription_init_default(&_subscriber, &_node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "stepper_pan") != RCL_RET_OK) {
    LOGI(TAG, "rclc_subscription_init_default failed: try again later...");
    rcutils_reset_error();
    rmw_context_t* rmw_context = rcl_context_get_rmw_context(&_support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    RCNOCHECK(rcl_subscription_fini(&_subscriber, &_node));
    RCNOCHECK(rcl_node_fini(&_node));
    RCNOCHECK(rclc_support_fini(&_support));
    // Try again later
    Task* initDelayedROSTask = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _initROS(); }, _scheduler, false, NULL, NULL, true);
    initDelayedROSTask->enableDelayed(1000);
    return;
  } else {
    LOGI(TAG, "rclc_subscription_init_default succeeded!");
  }

  if (rclc_executor_init(&_executor, &_support.context, 1, &_allocator) != RCL_RET_OK) {
    LOGI(TAG, "rclc_executor_init failed: try again later...");
    rcutils_reset_error();
    rmw_context_t* rmw_context = rcl_context_get_rmw_context(&_support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    RCNOCHECK(rclc_executor_fini(&_executor));
    RCNOCHECK(rcl_subscription_fini(&_subscriber, &_node));
    RCNOCHECK(rcl_node_fini(&_node));
    RCNOCHECK(rclc_support_fini(&_support));
    // Try again later
    Task* initDelayedROSTask = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _initROS(); }, _scheduler, false, NULL, NULL, true);
    initDelayedROSTask->enableDelayed(1000);
    return;
  } else {
    LOGI(TAG, "rclc_executor_init succeeded!");
  }

  if (rclc_executor_add_subscription(&_executor, &_subscriber, &_msg, [](const void* msg_in) { roscom._stepper_command_callback(msg_in); }, ON_NEW_DATA) != RCL_RET_OK) {
    LOGI(TAG, "rclc_executor_add_subscription failed: try again later...");
    rcutils_reset_error();
    rmw_context_t* rmw_context = rcl_context_get_rmw_context(&_support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
    RCNOCHECK(rclc_executor_fini(&_executor));
    RCNOCHECK(rcl_subscription_fini(&_subscriber, &_node));
    RCNOCHECK(rcl_node_fini(&_node));
    RCNOCHECK(rclc_support_fini(&_support));
    // Try again later
    Task* initDelayedROSTask = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _initROS(); }, _scheduler, false, NULL, NULL, true);
    initDelayedROSTask->enableDelayed(1000);
    return;
  } else {
    LOGI(TAG, "rclc_executor_add_subscription succeeded!");
  }

  // Start ROS-Task
  LOGD(TAG, "starting _spinROSTask");
  _spinROSTask = new Task(200, TASK_FOREVER, [&] { _spinROS(); }, _scheduler, false, NULL, NULL, true);
  _spinROSTask->enableDelayed(1000);

  // Whishful thinking ahead
  _rosComState = RosComState::AGENT_CONNECTED;

  // execute callback (from website)
  if (_rosEventCallback != nullptr) {
    JsonDocument jsonMsg;
    jsonMsg["type"] = "ros_state";
    jsonMsg["state"] = getRosComState_as_string().c_str();
    jsonMsg.shrinkToFit();
    _rosEventCallback(jsonMsg);
  }
}

void RosCom::_spinROS() {
  switch (_rosComState) {
    case RosComState::AGENT_CONNECTED: {
      _rosComState = (RMW_RET_OK == rmw_uros_ping_agent(1, 10)) ? RosComState::AGENT_CONNECTED : RosComState::AGENT_DISCONNECTED;
      if (_rosComState == RosComState::AGENT_CONNECTED) {
        if (rclc_executor_spin_some(&_executor, RCL_MS_TO_NS(100)) == RCL_RET_ERROR) {
          LOGE(TAG, "rclc_executor_spin_some failed");
        }
      } else {
        LOGW(TAG, "Cannot ping agent!");
      }
    } break;
    case RosComState::AGENT_DISCONNECTED: {
      // Destroy the remains
      LOGI(TAG, "Clean-up befor reconnecting to agent!");
      rmw_context_t* rmw_context = rcl_context_get_rmw_context(&_support.context);
      (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
      RCNOCHECK(rclc_executor_fini(&_executor));
      RCNOCHECK(rcl_subscription_fini(&_subscriber, &_node));
      RCNOCHECK(rcl_node_fini(&_node));
      RCNOCHECK(rclc_support_fini(&_support));
      _rosComState = RosComState::WAITING_AGENT;

      // Stop the _spinROS-Task (will be done in _init)
      // Re-Init uROS (will re-start the _spinROS-Task)
      Task* initDelayedROSTask = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _initROS(); }, _scheduler, false, NULL, NULL, true);
      initDelayedROSTask->enableDelayed(1000);
    } break;
  }
}

void RosCom::_stepper_command_callback(const void* msg_in) {
  auto msg = static_cast<const std_msgs__msg__Int32*>(msg_in);
  // LOGI(TAG, "Received command: %d", msg->data);

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
