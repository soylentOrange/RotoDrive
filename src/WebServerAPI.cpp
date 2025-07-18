// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Copyright (C) 2025 Robert Wendlandt
 */

#include <Ticker.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <esp_timer.h>
#include <thingy.h>

#define TAG "WebServer"

void WebServerAPI::begin(Scheduler* scheduler) {
  // Just to be sure that the static webserver is not running anymore before being started (again)
  _webServer->end();

  // Task handling
  _sr.setWaiting();
  _scheduler = scheduler;

  // create and run a task for setting up the webserver
  Task* webServerTask = new Task(TASK_IMMEDIATE, TASK_ONCE, [&] { _webServerCallback(); }, _scheduler, false, NULL, NULL, true);
  webServerTask->enable();
}

void WebServerAPI::end() {
  LOGD(TAG, "Disabling WebServerAPI-Task...");
  LittleFS.end();
  _sr.setWaiting();
  _webServer->end();
  LOGD(TAG, "...done!");
}

void WebServerAPI::restart(uint32_t delayMillisBeforeRestartMillis) {
  _delayedTask.detach();
  if (delayMillisBeforeRestartMillis == 0) {
    LOGW(TAG, "Restart!");
    esp_restart();
  } else {
    LOGW(TAG, "Restart in %" PRIu32 " ms...", delayMillisBeforeRestartMillis);
    _delayedTask.once_ms(delayMillisBeforeRestartMillis, esp_restart);
  }
}

bool WebServerAPI::restartFactory(const char* partitionName, uint32_t delayMillisBeforeRestartMillis) {
  const esp_partition_t* partition = esp_partition_find_first(esp_partition_type_t::ESP_PARTITION_TYPE_APP, esp_partition_subtype_t::ESP_PARTITION_SUBTYPE_APP_FACTORY, partitionName);
  if (partition) {
    LOGW(TAG, "Set boot partition to %s", partitionName);
    ESP_ERROR_CHECK(esp_ota_set_boot_partition(partition));
    restart(delayMillisBeforeRestartMillis);
    return true;
  } else {
    ESP_LOGE(TAG, "Partition not found: %s", partitionName);
    return false;
  }
}

// Start the webserver
void WebServerAPI::_webServerCallback() {
  LOGD(TAG, "Starting WebServerAPI...");

  // Start Filesystem
  if (!LittleFS.begin(false)) {
    LOGE(TAG, "An Error has occurred while mounting LittleFS!");
  } else {
    LOGD(TAG, "LittleFS mounted!");
    _fsMounted = true;
  }

  // Handle getting files from File System (will auto-magically serve the gzipped files)
  _webServer->serveStatic("/", LittleFS, "/")
    .setCacheControl("max-age=600")
    .setFilter([&](__unused AsyncWebServerRequest* request) { return _fsMounted; });

  // clear persisted wifi config
  _webServer->on("/api/system/clearwifi", HTTP_POST, [&](AsyncWebServerRequest* request) {
    LOGW(TAG, "Clearing WiFi configuration...");
    espNetwork.clearConfiguration();
    LOGW(TAG, TAG, "Restarting!");
    auto* response = request->beginResponse(200, "text/plain", "WiFi credentials are gone! Restarting now...");
    request->send(response);
    restart(1000);
    led.setMode(LED::LEDMode::WAITING_CAPTIVE);
  });

  // do restart
  _webServer->on("/api/system/restart", HTTP_POST, [&](AsyncWebServerRequest* request) {
    LOGW(TAG, "Restarting!");
    auto* response = request->beginResponse(200, "text/plain", "Restarting now...");
    request->send(response);
    stepper.end();
    restart(1000);
    led.setMode(LED::LEDMode::WAITING_WIFI);
  });

  // reboot esp into SafeBoot
  _webServer->on("/api/system/safeboot", HTTP_POST, [&](AsyncWebServerRequest* request) {
    LOGW(TAG, "Restarting in SafeBoot-mode...");
    if (restartFactory("safeboot", 1000)) {
      auto* response = request->beginResponse(200, "text/plain", "Restarting into SafeBoot now...");
      request->send(response);
      stepper.end();
      led.setMode(LED::LEDMode::NONE);
    } else {
      LOGW(TAG, "SafeBoot-partition not found");
      auto* response = request->beginResponse(502, "text/plain", "SafeBoot partition not found!");
      request->send(response);
    }
  });

  // Set 404-handler only when the captive portal is not shown
  if (eventHandler.getNetworkState() != Mycila::ESPConnect::State::PORTAL_STARTED) {
    LOGD(TAG, "Register 404 handler in WebServerAPI");
    _webServer->onNotFound([](AsyncWebServerRequest* request) {
      LOGW(TAG, "Send 404 on request for %s", request->url().c_str());
      request->send(404);
    });
  } else {
    LOGD(TAG, "Skip registering 404 handler in WebServerAPI");
  }

  _webServer->begin();

  LOGD(TAG, "...done!");
  _sr.signalComplete();
}

StatusRequest* WebServerAPI::getStatusRequest() {
  return &_sr;
}
