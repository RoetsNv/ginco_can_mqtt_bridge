#include "GincoBridge.h"

GincoBridge::GincoBridge(byte moduleID, String friendly_name,PubSubClient* client):
    moduleID(moduleID),
    friendly_name(friendly_name),
    mqtt_client(client)
{

    for(int i=0;i<8;i++){
        this->data_buffer[i]=0x00;
    }
    this->heartbeat_interval=2000;
    this->now=millis();
    this->can_controller=GCANController(this->moduleID);
}

void GincoBridge::long_to_data_buffer(long input){
  
  for (int i = 0; i < 4; i++)
  {
    data_buffer[i] = ((input >> (8 * i)) & 0XFF);
  }
}
void GincoBridge::identify(){
    Serial.print("I am: ");Serial.print(this->friendly_name);Serial.print(" ID: ");Serial.println(this->moduleID);
}
void GincoBridge::on_can_msg(GCanMessage g){
    this->to_sendJSON.clear();
    this->to_sendJSON["extended_id"]= g.extended_id;
    this->to_sendJSON["event"]= g.event;
    this->to_sendJSON["source_module_id"]= g.source_module_id;
    this->to_sendJSON["linked"]= g.linked;
    this->to_sendJSON["ack"]= g.ack;
    this->to_sendJSON["feature_type"]= g.feature_type;
    this->to_sendJSON["index"]= g.index;
    this->to_sendJSON["function_address"]= g.function_address;
    this->to_sendJSON["buffer_size"]= g.buffer_size;
    this->to_sendJSON["received_long"]= g.received_long;
    char msgString[256];
    serializeJson(this->to_sendJSON, msgString,256);
    this->topic_string[20] = "0123456789ABCDEF"[g.source_module_id >> 4];
    this->topic_string[21] = "0123456789ABCDEF"[g.source_module_id & 0x0F];
    Serial.print("publishing to : ");Serial.println(this->topic_string);
    this->mqtt_client->publish(this->topic_string, msgString,256);
}

void GincoBridge::send_can_msg(GCanMessage m){
    this->long_to_data_buffer(m.received_long);
    Serial.println("in bridge, sending to controller");
    this->can_controller.send_can_msg(this->can_controller.give_can_id(m.event,m.source_module_id, m.feature_type,m.index,m.function_address,m.ack),data_buffer,m.buffer_size);
}

void GincoBridge::loop(){
    //loop can-driver
    this->can_controller.check_can_bus();
    //check if there are any can msg's ready
    if (this->can_controller.gcan_received()){
        this->on_can_msg(this->can_controller.give_last_msg());
    }
}