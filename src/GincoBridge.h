#include <stdlib.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include <iostream>
#include <vector>
using namespace std;

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
        StaticJsonDocument<256> to_sendJSON; // buffers used to publish/receive mqtt messages
        StaticJsonDocument<256> receivedJSON;
        PubSubClient *mqtt_client; // MQTT client to talk to
        Preferences flash; // Persistant storage ESP32
        vector<vector<long>>scene_triggers; //RAM access to increase performance:  first index -> index in list of scenes ; second index -> list of triggers that trigger that scene
        vector<vector<long>>toggle_scene_triggers; //first index -> scene group index ; second index -> list of triggers that trigger the scene cycling
        uint16_t group_data[15][15];
        

    public:
        int output_state[7];
        GincoBridge(byte moduleID,String friendly_name,PubSubClient* client);
        void init();
        void long_to_data_buffer(long input);
        void on_can_msg(GCanMessage m);
        void send_can_msg(GCanMessage m);
        void write_scene(StaticJsonDocument<1024> scene_json);
        void check_scenes(long canID);
        void clear_data_buffer();
        void activate_scene(uint16_t index);
        void cycle_scene_group(uint16_t group_id);
        void flash_to_ram();
        void identify();
        void bridge_control(uint16_t type);
        void loop();

};