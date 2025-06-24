// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2025 Robert Wendlandt
 */
#pragma once

#include <TaskSchedulerDeclarations.h>
#include <micro_ros_platformio.h>
//#include <pthread.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>

class RosCom {
  public:
    RosCom() {}
    void begin(Scheduler* scheduler);
    void end();

  private:
    Scheduler* _scheduler = nullptr;
    void _initROS();
    rcl_allocator_t _allocator;
};
