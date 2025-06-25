// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2025 Robert Wendlandt
 */
#pragma once

#include <TaskSchedulerDeclarations.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>

#include <functional>
#include <map>
#include <string>

#define COMMAND_INVALID (-2)
#define COMMAND_STOP    (0)
#define COMMAND_FWD     (1)
#define COMMAND_REV     (-1)
#define COMMAND_HOME    (2)

class RosCom {
  public:
    enum class RosComState {
      UNKNOWN,
      CONNECTING,
      CONNECTED,
      ERROR
    };

  private:
    std::map<RosComState, std::string> RosComState_string_map = {
      {RosComState::UNKNOWN, "UNKNOWN"},
      {RosComState::CONNECTING, "CONNECTING"},
      {RosComState::CONNECTED, "CONNECTED"},
      {RosComState::ERROR, "ERROR"}};

  public:
    RosCom() {}
    void begin(Scheduler* scheduler);
    void end();
    RosComState getRosComState() { return _rosComState; }
    std::string getRosComState_as_string() { return RosComState_string_map[_rosComState]; }
    typedef std::function<void(JsonDocument doc)> RosEventCallback;
    void listenRosEvent(RosEventCallback callback) { _rosEventCallback = callback; }

  private:
    Scheduler* _scheduler = nullptr;
    RosComState _rosComState = RosComState::UNKNOWN;
    void _initROS();
    Task* _spinROSTask = nullptr;
    void _spinROS();
    // Task* _checkROSTask = nullptr;
    // void _checkROS();
    rcl_allocator_t _allocator;
    rclc_support_t _support;
    std_msgs__msg__Int32 _msg;
    rclc_executor_t _executor;
    rcl_node_t _node;
    rcl_subscription_t _subscriber;
    void _stepper_command_callback(const void* msg_in);
    int32_t _lastCommand = COMMAND_INVALID;
    // uint32_t _lastPing = 0;
    // to be called by stepper for motor specific events
    RosEventCallback _rosEventCallback = nullptr;
};
