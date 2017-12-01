#include "libs/arduino/PubNub.h"
#include <SPI.h>
#include <Ethernet.h>


class iotbridge{
  public:
  
  const char *publish_key;
  const char *subscribe_key;
  const char *uuid;
  const char *channel;
  const char *message;
  
  bool init(const char *publish_key, const char *subscribe_key, const char *uuid);
  bool send(const char *channel, const char *message);
  String connect(const char *channel);
  
};
  
bool iotbridge::init(const char *publish_key, const char *subscribe_key, const char *uuid){
  
  PubNub.begin(publish_key,subscribe_key);
  PubNub.set_uuid(uuid);
}

bool iotbridge::send(const char *channel, const char *message){
   WiFiEspClient *client;
  client = PubNub.publish(channel,message);
  return client;
}

String iotbridge::connect(const char *channel){
  
  String message = "";
  int i = 0;
  PubSubClient *pclient = PubNub.subscribe(channel);
  if (!pclient) {
    return "";
  }
  while (pclient->wait_for_data()) {
    char c = pclient->read();
    message += c;
  }

  pclient->stop();
  Serial.println();
  return message;
  
}
