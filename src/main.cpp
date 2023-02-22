#include <Arduino.h>
#include "controllers/MQTT_com.h"
//wifi config        
WiFiClient espClient;
PubSubClient client(espClient);
GincoBridge gb = GincoBridge(0x00,"main_bridge", &client);
void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    init_wifi();
}

void loop() {
  gb.loop();
  if (!client.connected()) {
      reconnect();
    }
  client.loop();
}