#ifndef _WIFI_HANDLER_H
#define _WIFI_HANDLER_H

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <functional>

namespace wifi_handler
{
    void setWiFiMACAddress(const uint8_t mac[6]);
    void setOTAOnStart(std::function<void(void)> fn);
    void initWiFi(String ssid, String password);
}

#endif