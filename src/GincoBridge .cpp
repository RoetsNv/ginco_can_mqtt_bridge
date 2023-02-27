#include "GincoBridge.h"

GincoBridge::GincoBridge(byte moduleID, String friendly_name,PubSubClient* client):
    moduleID(moduleID),
    friendly_name(friendly_name),
    mqtt_client(client)
{

    this->clear_data_buffer();
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
void GincoBridge::clear_data_buffer(){
    for(int i=0;i<8;i++){
        this->data_buffer[i]=0x00;
    }
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

void GincoBridge::write_scene(StaticJsonDocument<256> scene_json){
    this->flash.begin("saved_scenes", false);
    unsigned int scene_count = this->flash.getUInt("scounter", 0);
    String key= "name";
    Serial.println(key+scene_count);
    String data= scene_json["to_save"];
    this->flash.putString((key+scene_count).c_str(),data);
    //put trigger ID's
    key="trigger";
    long canID;
    int count_triggers=0;
    while(true){
        canID=scene_json["triggers"][count_triggers];
        if(canID==0){
            break;
        }
        else{
            count_triggers++;
        }
    }
    int id_arr_size=4*count_triggers;
    id_arr_size = (id_arr_size>80)? 80:id_arr_size;
    byte t[4];
    byte tt[id_arr_size];
    for(int i= 0; i<count_triggers;i++){
        canID=scene_json["triggers"][i];
        memcpy(t,&canID,4);
        int memindex=0;
        Serial.println("writing: " +canID);
        for(int j= 0; j<4;j++){
            memindex=(4*i)+j;
            Serial.print(t[j],HEX);Serial.print(" on spot: "); Serial.println(memindex);
            tt[memindex]=t[j];
        }
    }
    this->flash.putBytes((key+scene_count).c_str(),tt,id_arr_size);
    //Put group
    int groupID=scene_json["group"];
    key="group";
    this->flash.putInt((key+scene_count).c_str(),groupID);
    //Put group index
    key="group_index";
    groupID=scene_json["group_index"];
    this->flash.putInt((key+scene_count).c_str(),groupID);
    //put action ID's
    key="action";
    count_triggers=0;
    while(true){
        canID=scene_json["actions"][count_triggers];
        if(canID==0){
            break;
        }
        else{
            count_triggers++;
        }
    }
    id_arr_size=4*count_triggers;
    id_arr_size = (id_arr_size>80)? 80:id_arr_size;
    byte d[4];
    byte ta[id_arr_size];
    byte td[id_arr_size];
    long payload_data;
    for(int i= 0; i<count_triggers;i++){
        canID=scene_json["actions"][i];
        payload_data=scene_json["actions_data"][i];
        memcpy(t,&canID,4);
        memcpy(d,&payload_data,4);
        int memindex=0;
        for(int j= 0; j<4;j++){
            memindex=(4*i)+j;
            tt[memindex]=t[j];
            td[memindex]=d[j];
        }
    }
    this->flash.putBytes((key+scene_count).c_str(),ta,id_arr_size);
    key="action_data";
    this->flash.putBytes((key+scene_count).c_str(),td,id_arr_size);
    this->flash.end();
}
long buff_to_long(byte*b,int i){
    long result=0;
    result = 0;
    result |= (long)b[((4*i)+3)] << 24;
    result |= (long)b[((4*i)+2)] << 16;
    result |= (long)b[((4*i)+1)] << 8;
    result |= b[(4*i)];
    return result;
}
void GincoBridge::check_scenes(long canID){
    this->flash.begin("saved_scenes", true);
    unsigned int scene_count = (this->flash.getUInt("scounter", 0))/4;
    String key= "triggers";
    long mem_canID;
    int arr_size;
    for(int i=0;i<scene_count;i++){
        arr_size= (this->flash.getBytesLength((key+i).c_str()));
        byte tt[arr_size];
        this->flash.getBytes((key+i).c_str(),tt,arr_size);
        //convert bytes to long
        mem_canID = buff_to_long(tt,i);
        if(mem_canID==canID){
            //Its a match!
            key="actions";
            arr_size=this->flash.getBytesLength((key+i).c_str());
            byte ta[arr_size];
            byte td[arr_size];
            this->flash.getBytes((key+i).c_str(),ta,arr_size);
            key="actions_data";
            this->flash.getBytes((key+i).c_str(),td,arr_size);
            //4 bytes per long;
            arr_size = arr_size/4;
            for(int j=0;j<arr_size;j++){
                //fill data buffer with payload
                this->clear_data_buffer();
                for(int data_index=0;data_index<4;data_index++){
                   data_buffer[data_index]= td[((4*j)+data_index)];     
                }
                //send action on bus
                this->can_controller.send_can_msg(buff_to_long(tt,i),data_buffer,8);
            }
        }
    }
    this->flash.end();
}

void GincoBridge::loop(){
    //loop can-driver
    this->can_controller.check_can_bus();
    //check if there are any can msg's ready
    if (this->can_controller.gcan_received()){
        this->on_can_msg(this->can_controller.give_last_msg());
    }
}