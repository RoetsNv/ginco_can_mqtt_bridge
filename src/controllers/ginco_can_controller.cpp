#include "ginco_can_controller.h"

GCANController::GCANController(byte moduleID):
moduleID(moduleID)
{
    CAN.setPins(35, 5);
    // start the CAN bus at 500 kbps
    if (!CAN.begin(500E3)) {
        Serial.println("Starting CAN failed!");
        while (1);
    }
    for(int i = 0; i<10;i++){
        this->interested_in[i]=0x00;
    }
    this->last_msg= new GCanMessage();
}
GCANController::GCANController()
{
    CAN.setPins(35, 5);
    // start the CAN bus at 500 kbps
    if (!CAN.begin(500E3)) {
        Serial.println("Starting CAN failed!");
        while (1);
    }
}

long GCANController::give_can_id(boolean is_event, byte targetmoduleID,byte feature_type,byte index_number,byte func_id,boolean ack){
  //Event: 0x4000000 Action: 0x00
  long can_id= (is_event) ? 0x4000000 : 0x00;
  can_id+= (ack) ? 0x10000: 0x00;
  //Set module id
  long mod_id=0;mod_id+=targetmoduleID;
  mod_id=mod_id<<18;
  can_id=can_id+mod_id;
  //Set 8 bit feature address
  feature_type=feature_type<<5;
  feature_type+=index_number;
  long feat_interf=0;feat_interf+=feature_type;
  feat_interf=feat_interf<<8;
  feat_interf+=func_id;
  can_id+=feat_interf;
  char buffer[15];
  ltoa(can_id, buffer, 16);
  Serial.println(buffer);
  return can_id;
}

void GCANController::send_can_msg(long can_id,const byte *data,size_t buffer_size){
  Serial.print("in controller canid: ");Serial.println(can_id);
   CAN.beginExtendedPacket(can_id);
   CAN.write(data,buffer_size);
   CAN.endPacket();
   Serial.print("Done Writing");
}

void GCANController::add_moduleID(byte moduleID){
    for(int i = 0; i<10;i++){
        if(this->interested_in[i]==0x00){
            this->interested_in[i]=moduleID;
        }
    }
}


///////-----------------------------RECEIVE-------------------------------------------------------

//Receiver helpers

boolean filter_msg(long can_id,byte *interested_in){
    //Mask to extract module ID
    long module_id= (can_id >> 18) & 0xFF;
    for(int i=0; i<10;i++){
    if(module_id==interested_in[i]){
        //MATCH! this module is interested in messages from this source
        //Serial.println("I'm very interested");
        return 1;
        } 
    }
    return 0;
}
GCanMessage* parse_message(long can_id,size_t buf_size,long received_long){
      GCanMessage* msg_ptr = new GCanMessage();
      msg_ptr->extended_id=can_id;
      msg_ptr->event=(can_id >> 26) & 0x01;
      msg_ptr->source_module_id=(can_id >> 18) & 0xFF;
      msg_ptr->linked=(can_id >> 17) & 0x01;
      msg_ptr->ack=(can_id >> 16) & 0x01;
      msg_ptr->feature_type=(can_id >> 13) & 0x07;
      msg_ptr->index=(can_id >> 8) & 0x1F;
      msg_ptr->function_address=can_id & 0xFF;
      msg_ptr->buffer_size=buf_size;
      msg_ptr->received_long=received_long;
      return msg_ptr;
}
//------Parsers----------//
String parse_feature(byte feature_id){
  switch (feature_id) {
  case 0x00:
    return "Button";
    break;
  case 0x01:
    return "Switch out";
    break;
  case 0x02:
    return "Dimmer";
    break;
  default:
    return "";
    break;
  }
}
String parse_function(byte function_id){
  switch (function_id) {
  case 0x00:
    return "Single Click";
    break;
  case 0x01:
    return "Double Click";
    break;
  case 0x02:
    return "Long Press";
    break;
  case 0x03:
    return "Long Press Release";
    break;
  default:
    return "";
    break;
  }
}
void print_message(GCanMessage *msg){
  if(msg->event){Serial.println("This is a Event Message");}else{Serial.println("This is a Action Message");};
  Serial.print("It comes from module: ");Serial.println(msg->source_module_id);
  Serial.print("Linked and ACK: ");Serial.print(msg->linked);Serial.println(msg->ack);
  Serial.print("Feature: ");Serial.print(parse_feature(msg->feature_type));Serial.print(" with index: ");Serial.println(msg->index);
  Serial.print("It logged the function: ");Serial.println(parse_function(msg->function_address));
  return;
}

void GCANController::handle_can_msg(int packet_size){

    if (CAN.packetRtr()) {
      // Remote transmission request, packet contains no data
      Serial.print("RTR ");
      return;
    }
    Serial.print("Received ");
    Serial.print("packet with id 0x");
    long packetID=CAN.packetId();
    Serial.print(packetID, HEX);
    if (CAN.packetRtr()) {
    Serial.print(" and requested length ");
    Serial.println(CAN.packetDlc());
    } else {
      Serial.print(" and length ");
      Serial.println(packet_size);  
      int counter=0;
      while (CAN.available()) {
        receive_buffer[counter]=CAN.read();
        //Serial.print(output_buffer[counter],HEX);Serial.print("-");
        counter++;
      }
      long received_long=0;
      for (int i = 0; i < packet_size; i++)
      {
          auto byteVal = (((long)receive_buffer[i]) << (8 * i));
          received_long |= byteVal;
      }
    Serial.println(received_long);
    //Free up memory allocated by previous msg
    delete this->last_msg;
    this->last_msg=parse_message(packetID,packet_size,received_long);
    print_message(this->last_msg);
    this->gcan_ready=true;
    }    

}
void GCANController::check_can_bus(){
    int packet_size=CAN.parsePacket();
    if(packet_size!=0){
        this->handle_can_msg(packet_size);
    }
    else{
        return;
    }
}

boolean GCANController::gcan_received(){
  return this->gcan_ready;
}
GCanMessage GCANController::give_last_msg(){
  GCanMessage m = *this->last_msg;
  this->gcan_ready=false;
  return m;
}