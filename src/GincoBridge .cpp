#include "GincoBridge.h"
//----------------globals and tools--------------------------------------
long buff_to_long(byte*b,int i){
    long result=0;
    result = 0;
    result |= (long)b[((4*i)+3)] << 24;
    result |= (long)b[((4*i)+2)] << 16;
    result |= (long)b[((4*i)+1)] << 8;
    result |= b[(4*i)];
    return result;
}
uint8_t group_cycle_index[15]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//----------------constructor + member functions--------------------------------------
GincoBridge::GincoBridge(byte moduleID, String friendly_name,PubSubClient* client):
    moduleID(moduleID),
    friendly_name(friendly_name),
    mqtt_client(client)
{

    this->clear_data_buffer();
    this->heartbeat_interval=2000;
    this->now=millis();
    this->can_controller=GCANController(this->moduleID);
    this->flash_to_ram();

}
void GincoBridge::flash_to_ram(){
    if(this->scene_triggers != nullptr){
        //first delete possible allocated memory
        for(uint16_t i=0;i<sizeof(this->scene_triggers);i++){
            if(this->scene_triggers[i] != nullptr){
                delete[] this->scene_triggers[i];
            }
        }
        delete[]this->scene_triggers;
    }
    for(uint16_t i=0;i<15;i++){
        if(this->toggle_scene_triggers[i] != nullptr){
            delete[] this->toggle_scene_triggers[i];
        }
    }
    this->flash.begin("saved_scenes", true);
    int scene_count = this->flash.getInt("scounter", -1); //defaults to -1 to detect if no scene or group has been set.
    if(scene_count>=0){
        this->scene_triggers=new long*[scene_count];
        String key= "triggers";
        int arr_size;
        for(uint16_t i=0;i<scene_count;i++){
            key= "triggers";
            arr_size= (this->flash.getBytesLength((key+i).c_str()));
            this->scene_triggers[i]=new long[(arr_size/4)];
            byte tt[arr_size];
            this->flash.getBytes((key+i).c_str(),tt,arr_size);
            //convert bytes to long
            for(uint16_t trigger_index=0;trigger_index<(arr_size/4);trigger_index++){
            this->scene_triggers[i][trigger_index] = buff_to_long(tt,trigger_index);
            }
            //Save index to group for later reference;
            key="group";
            int groupid=this->flash.getInt((key+scene_count).c_str(),-1);
            if(groupid>=0){
                key="group_index";
                int group_index=this->flash.getInt((key+scene_count).c_str(),-1);
                if(group_index>=0){
                    this->group_data[groupid][group_index] = scene_count;
                }
            }
        }
        this->flash.end();
        this->flash.begin("scene_groups", true);
        int group_count = this->flash.getInt("gcounter", -1);
        if(group_count>0){
            for(uint16_t i=0;i<group_count;i++){
                key= "triggers";
                arr_size= (this->flash.getBytesLength((key+i).c_str()));
                this->toggle_scene_triggers[i]=new long[(arr_size/4)];
                byte tt[arr_size];
                this->flash.getBytes((key+i).c_str(),tt,arr_size);
                //convert bytes to long
                for(uint16_t trigger_index=0;trigger_index<(arr_size/4);trigger_index++){
                this->toggle_scene_triggers[i][trigger_index] = buff_to_long(tt,trigger_index);
                }
            }
        }
        
        this->flash.end();
    }
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
    for(uint8_t i=0;i<8;i++){
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
    this->mqtt_client->publish(this->topic_string, msgString,256);
    Serial.print("In json: Can id is : ");Serial.println(g.extended_id);

}

void GincoBridge::send_can_msg(GCanMessage m){
    this->long_to_data_buffer(m.received_long);
    Serial.println("in bridge, sending to controller");
    this->can_controller.send_can_msg(this->can_controller.give_can_id(m.event,m.source_module_id, m.feature_type,m.index,m.function_address,m.ack),data_buffer,m.buffer_size);
}

void GincoBridge::write_scene(StaticJsonDocument<256> scene_json){
    int check_group=scene_json["actions"][0];
    String key= "name";
    long canID;
    int count_triggers=0;
    if(check_group == 0){
        //set group cycle
        this->flash.begin("scene_groups", false);
        int group_counter = this->flash.getInt("gcounter", -1) +1; //index should start at 0 --> +1 ; defaults to -1 to detect if no scene or group has been set.
        String data= scene_json["name"];
        this->flash.putString((key+group_counter).c_str(),data);
        key= "group_id";
        uint16_t gid=scene_json["group_id"];
        this->flash.putShort((key+group_counter).c_str(),gid);
        //put trigger ID's
        key="trigger";
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
        for(uint16_t i= 0; i<count_triggers;i++){
            canID=scene_json["triggers"][i];
            memcpy(t,&canID,4);
            uint16_t memindex=0;
            Serial.println("writing: " +canID);
            for(uint16_t j= 0; j<4;j++){
                memindex=(4*i)+j;
                //Serial.print(t[j],HEX);Serial.print(" on spot: "); Serial.println(memindex);
                tt[memindex]=t[j];
            }
        }
        this->flash.putBytes((key+group_counter).c_str(),tt,id_arr_size);
        this->flash.end();

    }
    else{
        Serial.println("msg is a standard scene definition");
        // msg is a standard scene definition
        this->flash.begin("saved_scenes", false);
        int scene_count = this->flash.getInt("scounter", -1) +1; //index should start at 0 --> +1 
        Serial.print("scene index is: ");Serial.println(scene_count);
        key= "name";
        String data= scene_json["name"];
        Serial.print("name: ");Serial.println(data);
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
        Serial.print("Trigger size is : ");Serial.println(count_triggers);
        int id_arr_size=4*count_triggers;
        id_arr_size = (id_arr_size>80)? 80:id_arr_size;
        byte t[4];
        byte tt[id_arr_size];
        for(uint16_t i= 0; i<count_triggers;i++){
            canID=scene_json["triggers"][i];
            memcpy(t,&canID,4);
            uint16_t memindex=0;
            Serial.println("writing: " +canID);
            for(uint16_t j= 0; j<4;j++){
                memindex=(4*i)+j;
                Serial.print(t[j],HEX);Serial.print(" on spot: "); Serial.println(memindex);
                tt[memindex]=t[j];
            }
        }
        this->flash.putBytes((key+scene_count).c_str(),tt,id_arr_size);
        //Put group
        uint16_t gid=scene_json["group_id"];
        key="group_id";
        this->flash.putShort((key+scene_count).c_str(),gid);
        //Put group index
        key="group_index";
        gid=scene_json["group_index"];
        this->flash.putShort((key+scene_count).c_str(),gid);
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
                Serial.print("Action size is : ");Serial.println(count_triggers);
        id_arr_size=4*count_triggers;
        id_arr_size = (id_arr_size>80)? 80:id_arr_size;
        byte d[4];
        byte ta[id_arr_size];
        byte td[id_arr_size];
        long payload_data;
        for(uint16_t i= 0; i<count_triggers;i++){
            canID=scene_json["actions"][i];
            payload_data=scene_json["actions_data"][i];
            Serial.println("writing: " +canID);
            memcpy(t,&canID,4);
            memcpy(d,&payload_data,4);
            int memindex=0;
            for(uint8_t j= 0; j<4;j++){
                memindex=(4*i)+j;
                tt[memindex]=t[j];
                td[memindex]=d[j];
            }
        }
        this->flash.putBytes((key+scene_count).c_str(),ta,id_arr_size);
        key="action_data";
        this->flash.putBytes((key+scene_count).c_str(),td,id_arr_size);
    }
    this->flash.end();
    this->flash_to_ram();
}
void GincoBridge::check_scenes(long canID){
    for(uint16_t i=0;i<sizeof(this->scene_triggers);i++){
        for(uint16_t j=0;j<sizeof(this->scene_triggers[i]);j++){
            if(canID==this->scene_triggers[i][j]){
                //Its a match!
                this->activate_scene(i);
            }
        }
    }
    for(uint16_t i=0;i<sizeof(this->toggle_scene_triggers);i++){
        for(uint16_t j=0;j<sizeof(this->toggle_scene_triggers[i]);j++){
            if(canID==this->toggle_scene_triggers[i][j]){
                //Its a match!
                this->cycle_scene_group(i);
            }
        }
    }
}
void GincoBridge::cycle_scene_group(uint16_t group_id){
    if(group_id>14){return;}
    uint16_t index= group_cycle_index[group_id]+1;
    index = (index==15)? 0 : index;
    while(index<15){
        //check if index is set otherwise search for next candidate.
        if(this->group_data[group_id][index] != 256){
            break;
        }
        else{
            index++;
        }
    }
    if(index>14){return;}
    this->activate_scene(this->group_data[group_id][index]);
    return;
}
void GincoBridge::activate_scene(uint16_t i){ 
    this->flash.begin("saved_scenes", true);
    String key="actions";
    int arr_size=this->flash.getBytesLength((key+i).c_str());
    byte ta[arr_size];
    byte td[arr_size];
    //get action can id's
    this->flash.getBytes((key+i).c_str(),ta,arr_size);
    key="actions_data";
    //get actionData 
    this->flash.getBytes((key+i).c_str(),td,arr_size);
    //4 bytes per long;
    arr_size = arr_size/4;
    for(uint16_t j=0;j<arr_size;j++){
        //fill data buffer with payload
        this->clear_data_buffer();
        for(uint16_t data_index=0;data_index<4;data_index++){
            data_buffer[data_index]= td[((4*j)+data_index)];     
        }
        //send action on bus
        this->can_controller.send_can_msg(buff_to_long(ta,i),data_buffer,8);
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