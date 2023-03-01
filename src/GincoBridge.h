#include <stdlib.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <Preferences.h>

#include <controllers/ginco_can_controller.h>
class GincoBridge {
    private:
        //module specific
        byte moduleID;
        String friendly_name;
        String hw_version;
        char topic_string[23]="ginco_can_receive/0x";
        byte data_buffer[8];
        GCANController can_controller;
        const int timers[2]={50,500};
        int timer_ticks[2]={0,0};
        unsigned long now;
        unsigned long heartbeat_interval; //when heartbeats should be send
        StaticJsonDocument<256> to_sendJSON;
        StaticJsonDocument<256> receivedJSON;
        PubSubClient *mqtt_client;
        Preferences flash;
        long **scene_triggers;
        long **toggle_scene_triggers;

    public:
        int output_state[7];
        GincoBridge(byte moduleID,String friendly_name,PubSubClient* client);
        void init();
        void long_to_data_buffer(long input);
        void on_can_msg(GCanMessage m);
        void send_can_msg(GCanMessage m);
        void write_scene(StaticJsonDocument<256> scene_json);
        void check_scenes(long canID);
        void clear_data_buffer();
        void activate_scene(int index);
        void identify();
        void loop();

};