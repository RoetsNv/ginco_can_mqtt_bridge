#ifndef MQTT_com
#define MQTT_com
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include "Arduino.h"
#include "../GincoBridge.h"



extern GincoBridge gb;
extern PubSubClient client;
void setup_wifi();
void reconnect();
void callback(char* topic, byte* message, unsigned int length);
void send_data();
void init_wifi();
void constructJSON();
#endif 