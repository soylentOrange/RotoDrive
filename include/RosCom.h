// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2025 Robert Wendlandt
 */
#pragma once

#include <TaskSchedulerDeclarations.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
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
      WAITING_AGENT,
      AGENT_AVAILABLE,
      AGENT_CONNECTED,
      AGENT_DISCONNECTED
    };

  private:
    std::map<RosComState, std::string> RosComState_string_map = {
      {RosComState::WAITING_AGENT, "WAITING_AGENT"},
      {RosComState::AGENT_AVAILABLE, "AGENT_AVAILABLE"},
      {RosComState::AGENT_CONNECTED, "AGENT_CONNECTED"},
      {RosComState::AGENT_DISCONNECTED, "AGENT_DISCONNECTED"}};

  public:
    RosCom() {}
    // void begin(Scheduler* scheduler, IPAddress agent_ip, size_t agent_port);
    void begin(Scheduler* scheduler);
    void end();
    RosComState getRosComState() { return _rosComState; }
    std::string getRosComState_as_string() { return RosComState_string_map[_rosComState]; }
    typedef std::function<void(JsonDocument doc)> RosEventCallback;
    void listenRosEvent(RosEventCallback callback) { _rosEventCallback = callback; }

  private:
    Scheduler* _scheduler = nullptr;
    RosComState _rosComState = RosComState::AGENT_DISCONNECTED;
    void _initROS();
    Task* _spinROSTask = nullptr;
    void _spinROS();
    rcl_allocator_t _allocator;
    rclc_support_t _support;
    std_msgs__msg__Int32 _msg;
    rclc_executor_t _executor;
    rcl_node_t _node;
    rcl_subscription_t _subscriber;
    void _stepper_command_callback(const void* msg_in);
    int32_t _lastCommand = COMMAND_INVALID;
    // to be called by stepper for motor specific events
    RosEventCallback _rosEventCallback = nullptr;
    // Transport Callbacks
    static bool cw_transport_open(struct uxrCustomTransport* transport);
    static bool cw_transport_close(struct uxrCustomTransport* transport);
    static size_t cw_transport_write(struct uxrCustomTransport* transport, const uint8_t* buf, size_t len, uint8_t* errcode);
    static size_t cw_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* errcode);
};
