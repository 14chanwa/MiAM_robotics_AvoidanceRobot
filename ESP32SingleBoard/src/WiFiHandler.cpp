#include <WiFiHandler.hpp>
#include <esp_wifi.h>

namespace wifi_handler
{
    String ssid_;
    String password_;

    void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info)
    {
        log_i("Connected to AP successfully!");
    }

    void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info)
    {
        log_i("Got new IP address");
        Serial.println(WiFi.localIP());
    }

    void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info)
    {
        log_i("Disconnected from WiFi access point");
        log_i("WiFi lost connection. Reason: %d", info.wifi_sta_disconnected.reason);
        log_i("Trying to Reconnect");
        WiFi.begin(ssid_, password_);
    }

    void setWiFiMACAddress(const uint8_t mac[6])
    {
        esp_err_t res = esp_wifi_set_mac(WIFI_IF_STA, &mac[0]);
        log_i("Set new WiFi MAC Address");
        Serial.println(WiFi.macAddress());
    }

    void setOTAOnStart(std::function<void(void)> fn)
    {
        ArduinoOTA.onStart(fn);
    }

    void task_handle_ota(void *parameters)
    {
        for (;;)
        {
            ArduinoOTA.handle();
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
    }

    void initWiFi(String ssid, String password)
    {

        ssid_ = ssid;
        password_ = password;

        WiFi.mode(WIFI_STA);
        WiFi.onEvent(WiFiStationConnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
        WiFi.onEvent(WiFiGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);
        WiFi.onEvent(WiFiStationDisconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

        log_i("Current WiFi MAC Address: %c", WiFi.macAddress());

        WiFi.begin(ssid_, password_);
        Serial.print("Connecting to WiFi ..");
        while (WiFi.status() != WL_CONNECTED)
        {
            Serial.print('.');
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        Serial.println();
        Serial.print("Current ESP32 IP: ");
        Serial.println(WiFi.localIP());
        Serial.print("Gateway (router) IP: "); 
        Serial.println(WiFi.gatewayIP());
        Serial.print("Subnet Mask: "); 
        Serial.println(WiFi.subnetMask());
        Serial.print("Primary DNS: ");
        Serial.println(WiFi.dnsIP(0));
        Serial.print("Secondary DNS: "); 
        Serial.println(WiFi.dnsIP(1));

        ArduinoOTA.begin();

        xTaskCreate(
            task_handle_ota,
            "task_handle_ota",
            10000,
            NULL,
            70,
            NULL);
    }
}