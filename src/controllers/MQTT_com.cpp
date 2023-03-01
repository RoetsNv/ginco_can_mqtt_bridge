#include "MQTT_com.h"


const char* ssid = "WifiDink";
const char* password = "pw";
const char* mqtt_server = "192.168.1.89";
unsigned long now;
unsigned long lastMsg=0;
char msgString[250];
StaticJsonDocument<256> receivedMsg;

void print_m(GCanMessage *msg){
  if(msg->event){Serial.println("This is a Event Message");}else{Serial.println("This is a Action Message");};
  Serial.print("Module ID: ");Serial.println(msg->source_module_id);
  Serial.print("Feature: ");Serial.print(msg->feature_type);Serial.print(" with index: ");Serial.println(msg->index);
  Serial.print("It logged the function: ");Serial.println(msg->function_address);
  return;
}









void init_wifi(){
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
}
void reconnect() {
  if ( WiFi.status() != WL_CONNECTED) {
       WiFi.begin(ssid, password);
     while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        //If wifi cant connect check if pump has to run (failSafe)
     }
    }
  while (!client.connected()) {
    // Attempt to connect
    if (client.connect("ginco_mqtt_bridge","sj_mqtt","lala5123")) {
      // Subscribe
      client.subscribe("ginco_can_write");
      client.subscribe("ginco_scene_write");
    } else {
      delay(5000);
    }
  }
}
void callback(char* topic, byte* message, unsigned int length) {
  char messageTemp[length];
  for (int i = 0; i < length; i++) {
    messageTemp[i]= (char)message[i];
  }
  Serial.println("got message");
  //Serial.println(messageTemp);
   DeserializationError error = deserializeJson(receivedMsg, messageTemp);
    if (error) {  
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }
  else{
    if (strcmp(topic,"ginco_can_write") == 0){
      GCanMessage m;
      m.event=receivedMsg["event"];
      m.source_module_id=receivedMsg["source_module_id"];
      m.linked=receivedMsg["linked"];
      m.ack=receivedMsg["ack"];
      m.feature_type=receivedMsg["feature_type"];
      m.index=receivedMsg["index"];
      m.function_address=receivedMsg["function_address"];
      m.buffer_size=receivedMsg["buffer_size"];
      m.received_long=receivedMsg["received_long"];
      gb.identify();
      gb.send_can_msg(m);
    }
    else if (strcmp(topic,"ginco_scene_write") == 0){
      gb.write_scene(receivedMsg);
    }


    // String reqState= receivedMsg["state"];
    // int y=receivedMsg["brightness"];

    // if(reqState == "OFF"){
    //     dimmer.setState(OFF,receivedMsg["lamp_index"]);
    // }
    // else{
    //     if(y!=0){
    //         Serial.println("ON with brightness");
    //         dimmer.setPower(receivedMsg["brightness"],receivedMsg["lamp_index"]);
    //     }
    //     else{
    //         Serial.println("Just ON");
    //         dimmer.setState(ON,receivedMsg["lamp_index"]);
    //     }
    // }
  }
}
