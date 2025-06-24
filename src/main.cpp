// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2025 Robert Wendlandt
 */

#include <Preferences.h>
#include <TaskScheduler.h>
#include <thingy.h>
#include <micro_ros_platformio.h>

#define TAG "Main"

// Create the WebServer, ESPConnect, Task-Scheduler,... here
AsyncWebServer webServer(HTTP_PORT);
Scheduler scheduler;
ESPNetwork espNetwork(webServer);
EventHandler eventHandler(espNetwork);
WebServerAPI webServerAPI(webServer);
WebSite webSite(webServer);
LED led;
TwaiCan canBus(CAN_TX, CAN_RX);
Stepper stepper(canBus);
RosCom roscom;

// Allow logging for app via serial
#if defined(MYCILA_LOGGER_SUPPORT_APP)
Mycila::Logger* serialLogger = nullptr;
#endif

// Allow logging for app via webserial
#if defined(MYCILA_WEBSERIAL_SUPPORT_APP)
WebSerial webSerial;
Mycila::Logger* webLogger = nullptr;
#endif

void setup() {
  // Start Serial or USB-CDC
  #if !ARDUINO_USB_CDC_ON_BOOT
  Serial.begin(MONITOR_SPEED);
  // Only wait for serial interface to be set up when not using USB-CDC
  while (!Serial)
    continue;
  #else
  // USB-CDC doesn't need a baud rate
  Serial.begin();

  // Note: Enabling Debug via USB-CDC is handled via framework
  #endif

  // Set serial transport functions for uRos
  set_microros_serial_transports(Serial);

  // Add LED-Task to Scheduler
  led.begin(&scheduler);

  // Add ESPConnect-Task to Scheduler
  espNetwork.begin(&scheduler);

  // Add EventHandler to Scheduler
  eventHandler.begin(&scheduler);

  // Add WebServerAPI to Scheduler
  webServerAPI.begin(&scheduler);

  // Add WebSite to Scheduler
  webSite.begin(&scheduler);

  // Add Stepper to Scheduler
  stepper.begin(&scheduler);

  // Add ros to Scheduler
  roscom.begin(&scheduler);
}

void loop() {
  scheduler.execute();
}
