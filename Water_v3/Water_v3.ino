/*
    Water_v3
*/

#include <ESP8266WiFi.h>
#include <WiFiUDP.h>


#include <ArduinoJson.h>


const char* ssid = "DIR-300A";
const char* password = "8848evev";

unsigned long previousMillis = 0;
unsigned long previousMillis3 = 0;
const long interval = 8000;
const long interval2 = 16000;
IPAddress mip(239, 255, 255, 50);
String remote = "";
IPAddress remoteIp;

unsigned int localPort = 3234;     //udp resiver

char packetBuffer[UDP_TX_PACKET_MAX_SIZE];

WiFiServer server(4234);     //tcp resiver

WiFiUDP Udp;

WiFiClient clientS;

String ip = "";

boolean water1 = true;
boolean water2 = false;

boolean state = 0;
boolean stateL = 0;
boolean stateLL = 1;
int i = 0;


void setup() {
  pinMode(2, INPUT_PULLUP);
  Serial.begin(115200);
  delay(10);
  //sensors.begin();
  // Connect to WiFi network
  Serial.println();
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


  server.begin();
  //  Udp.begin(localPort);
  Udp.beginMulticast(WiFi.localIP(), mip, localPort);

  Serial.println("ok");
  ip = WiFi.localIP().toString();
  Serial.println(ip);
  stateL = digitalRead(2);
}



void loop() {
  handle_search();
  //handle_mes();
  handle_sendChData();
}

void handle_sendChData() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    state = !digitalRead(2);
    Serial.println(state);
    if (state) {
      if (water1) {
        water1 = false;
        water2 = true;
        if (remote != "") {
          Serial.println("234");
          DynamicJsonBuffer jsonBuffer;
          JsonObject& mes = jsonBuffer.createObject();
          DynamicJsonBuffer jsonBuffer2;
          JsonObject& body = jsonBuffer2.createObject();
          mes["ip"] = ip;
          mes["type"] = "post";
          mes["module"] = "water";

          Serial.println(state);
          body["water"] = String(state);

          mes["body"] = body;

          String output;
          mes.printTo(output);
          Serial.println(output);

          clientS.connect(remoteIp, 1337);
          clientS.print(output);
          clientS.stop();
          //      sendChData = false;
        }
      }
    }
    else {
      if(water2){
        water1 = true;
        water2 = false;
        if (remote != "") {
          Serial.println("234");
          DynamicJsonBuffer jsonBuffer;
          JsonObject& mes = jsonBuffer.createObject();
          DynamicJsonBuffer jsonBuffer2;
          JsonObject& body = jsonBuffer2.createObject();
          mes["ip"] = ip;
          mes["type"] = "post";
          mes["module"] = "water";

          Serial.println(state);
          body["water"] = String(state);

          mes["body"] = body;

          String output;
          mes.printTo(output);
          Serial.println(output);

          clientS.connect(remoteIp, 1337);
          clientS.print(output);
          clientS.stop();
          //      sendChData = false;
        }
      }
    }


  }
}

// handle udp search multicast

void handle_search() {
  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    remoteIp = Udp.remoteIP();
    remote = IpAddress2String(remoteIp);
    Serial.println(remote);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());

    // read the packet into packetBufffer
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    //Serial.println("Contents:  { \"MainArduino\":\"" + String(remote) + "\"":"");
    Serial.println(packetBuffer);

    clientS.connect(Udp.remoteIP(), 1337);
    clientS.print(String("{ \"type\":\"searchModule\",") + String(" \"module\":\"water\", ") + String(" \"ip\":\"") + ip + "\"}");
    clientS.stop();
  }
}



String IpAddress2String(const IPAddress & ipAddress)
{
  return String(ipAddress[0]) + String(".") + \
         String(ipAddress[1]) + String(".") + \
         String(ipAddress[2]) + String(".") + \
         String(ipAddress[3])  ;
}
