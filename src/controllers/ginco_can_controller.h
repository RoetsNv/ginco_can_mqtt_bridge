#include <stdlib.h>
#include <Arduino.h>
#include <CAN.h>
struct GCanMessage {
  long extended_id;
  boolean event;
  byte source_module_id;
  boolean linked; 
  boolean ack;
  byte feature_type;
  byte index;
  byte function_address;
  size_t buffer_size;
  long received_long;
  GCanMessage() {
      event=0;
      source_module_id=0;
      linked=0; 
      ack=0;
      feature_type=0;
      index=0;
      function_address=0;
      buffer_size=0;
      received_long=0;
  }
  GCanMessage(const GCanMessage &other) {
      event=other.event;
      source_module_id=other.source_module_id;
      linked=other.linked; 
      ack=other.ack;
      feature_type=other.feature_type;
      index=other.index;
      function_address=other.function_address;
      buffer_size=other.buffer_size;
      received_long=other.received_long;

  }
};
#include <functional>
#define GCAN_CALLBACK_SIGNATURE std::function<void(GCanMessage)> callback // callback(is_event,moduleID,is_linked,is_ack,feature,index,function_adr)


class GCANController {
    private:
    byte moduleID;
    byte output_buffer[8]; //buffer to write data to when sending msg
    byte receive_buffer[8];
    GCanMessage* last_msg;
    byte interested_in[10]; //list of module ID's the module should listen to
    boolean gcan_ready; // GCanMessage ready for module to read.

    GCAN_CALLBACK_SIGNATURE;
    public:
        GCANController(byte moduleID);
        GCANController();
        void setup();
        void send_can_msg(long can_id,const byte *data,size_t buffer_size);
        long give_can_id(boolean is_event, byte targetmoduleID,byte feature_type,byte index_number,byte func_id,boolean ack);
        void add_moduleID(byte moduleID); //add module ID to interested list
        void check_can_bus();
        boolean gcan_received();
        GCanMessage give_last_msg();
        void handle_can_msg(int packet_size);
}; 