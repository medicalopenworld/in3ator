
#ifndef _WIFI_OTA_H_
#define _WIFI_OTA_H_
// #define THINGSBOARD_ENABLE_DYNAMIC 1
#define THINGSBOARD_ENABLE_OTA 1

#include <Arduino.h>

#include "ThingsBoard.h"
#include "main.h"

#define WIFI_PUBLISH_INTERVAL 5000  // milliseconds

struct WIFIstruct {
  int provisioned = false;
  bool OTA_requested = false;
  bool provision_request_sent = false;
  bool provision_request_processed = false;
  bool serverConnectionStatus = false;
  bool lastServerConnectionStatus = false;
  bool lastWIFIConnectionStatus = false;
  bool lastOTAInProgress = false;
  long lastMQTTPublish = false;
  bool firstPublish = false;
  bool firstConfigPost = false;
  String device_token;
};

bool WIFIIsConnectedToServer();
bool WIFIIsConnected();
bool WIFICheckNewEvent();

#endif  // _WIFI_OTA_H_
