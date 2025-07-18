// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2025 Robert Wendlandt
 */
#pragma once

#include <ESPAsyncWebServer.h>
#include <TaskSchedulerDeclarations.h>
#include <Ticker.h>

class WebServerAPI {
  public:
    explicit WebServerAPI(AsyncWebServer& webServer) : _webServer(&webServer) { _sr.setWaiting(); }
    void begin(Scheduler* scheduler);
    void end();
    bool isFSMounted() { return _fsMounted; }
    StatusRequest* getStatusRequest();
    void restart(uint32_t delayMillisBeforeRestartMillis);
    bool restartFactory(const char* partitionName = "safeboot", uint32_t delayMillisBeforeRestartMillis = 0);

  private:
    void _webServerCallback();
    StatusRequest _sr;
    Scheduler* _scheduler = nullptr;
    AsyncWebServer* _webServer;
    bool _fsMounted = false;
    Ticker _delayedTask;
};
