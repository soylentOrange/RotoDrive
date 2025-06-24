// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2025 Robert Wendlandt
 */

// #include <micro_ros_platformio.h>
// #include <rcl/rcl.h>
// #include <rclc/executor.h>
// #include <rclc/rclc.h>
#include <thingy.h>
// #include <pthread.h>

#include <functional>

#define TAG "RosCom"

void RosCom::begin(Scheduler* scheduler) {
  // Task handling
  _scheduler = scheduler;
  set_microros_serial_transports(Serial);
  _allocator = rcl_get_default_allocator();

  // Set up a task for initializing the ros-communication
  Task* initROSTask = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _initROS(); }, _scheduler, false, NULL, NULL, true);
  initROSTask->enable();
  initROSTask->waitFor(webSite.getStatusRequest());
}

void RosCom::end() {
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
}
