/*
    Lamp_v3
*/

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
//#include <Ticker.h>/
//Ticker buttonsT;/

#include <ArduinoJson.h>

//#include <IRremoteESP/8266.h>
//IRsend irsend(13);/

//#include <pcf8574_esp.h>/
//#include <Wire.h>/

//TwoWire testWire;/

//PCF857x pcf8574(0x38/, &testWire);

const char* ssid = "DIR-300A";
const char* password = "8848evev";

unsigned long previousMillis = 0;
const long interval = 1000;

IPAddress mip(239, 255, 255, 50);
String remote = "";
IPAddress remoteIp;

unsigned int localPort = 3234;     //udp resiver

char packetBuffer[UDP_TX_PACKET_MAX_SIZE];

WiFiServer server(4234);     //tcp resiver

WiFiUDP Udp;

WiFiClient clientS;

String ip = "";

boolean chanleState[] = {0, 0, 0, 0};
boolean chanlePins[] = {5, 4, 14, 12};
boolean bwp[] = {1, 1, 1, 1};
boolean sendChData = false;

//void buttons () {
//  //boolean bip[] = {0, 0, 0, 0};
//
//  for (int i = 0; i < 4; i++) {
//    boolean bip = pcf8574.read(i);
//    if (bwp[i] && !bip) {
//      delay(10);
//      bip = pcf8574.read(i);
//      if (!bip) {
//        chanleState[i] = !chanleState[i];
//        digitalWrite(chanlePins[i], chanleState[i]);
//        sendChData = true;
//        Serial.println("button" + i);
//      }
//    }
//    bwp[i] = bip;
//  }
//}

void setup() {
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(D5, OUTPUT);
  pinMode(D6, OUTPUT);
  Serial.begin(115200);
  delay(10);
//  irsend.begi/n();
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
//  testWire.begin(0, /2);

  //testWire.setClock(100000L);
//  pcf8574.beg/in();
//  buttonsT.at/tach(0.1, buttons);
  Serial.println("ok");
  ip = WiFi.localIP().toString();
  Serial.println(ip);
}

void handle_mes() {
  WiFiClient client = server.available();
  if (client) {
    Serial.println("new req");

    String req = client.readStringUntil('\r');
    Serial.println(req);
    /*
      {"ip":"192.168.0.51", "module":"lamp", "type":"comande", "body":{"ch0":"0", "ch1":"1", "ch2":"0", "ch3":"0"}}
    */
    DynamicJsonBuffer jsonBuffer;

    JsonObject& root = jsonBuffer.parseObject(req);

    Serial.println("its my mes");
    String _type = "";
    _type = root["type"].as<String>();
    if (_type == "comande") {
      Serial.println("comande");
      chanleState[0] = root["body"]["ch0"].as<String>().toInt();
      chanleState[1] = root["body"]["ch1"].as<String>().toInt();
      chanleState[2] = root["body"]["ch2"].as<String>().toInt();
      chanleState[3] = root["body"]["ch3"].as<String>().toInt();
      Serial.println(chanleState[0]);
      Serial.println(chanleState[1]);
      Serial.println(chanleState[2]);
      Serial.println(chanleState[3]);
      setPin();
    } else if (_type == "comande_ir") {
      Serial.println("comande_ir");
      String code2 = root["body"]["ir"].as<String>();
      unsigned long code = strtoul(code2.c_str(), NULL, 10);
      Serial.println(code2);
      Serial.println(code, DEC);
///      irsend.sendNEC(code, 32);
    } else {

    }
    client.stop();
  }
}

void loop() {
  handle_search();
  handle_mes();
//  handle_sendChData();
}

//void handle_sendChData() {
//  unsigned long currentMillis = millis();
//  if (currentMillis - previousMillis >= interval) {
//    previousMillis = currentMillis;
//    if (sendChData) {
//      Serial.println("234");
//      DynamicJsonBuffer jsonBuffer;
//      JsonObject& mes = jsonBuffer.createObject();
//      DynamicJsonBuffer jsonBuffer2;
//      JsonObject& body = jsonBuffer2.createObject();
//      mes["ip"] = ip;
//      mes["type"] = "post";
//      mes["module"] = "lamp";
//
//      body["ch0"] = String(chanleState[0]);
//      body["ch1"] = String(chanleState[1]);
//      body["ch2"] = String(chanleState[2]);
//      body["ch3"] = String(chanleState[3]);
//      mes["body"] = body;
//
//      String output;
//      mes.printTo(output);
//      Serial.println(output);
//
//      clientS.connect(remoteIp, 1337);
//      clientS.print(output);
//      clientS.stop();
//      sendChData = false;
//    }
//  }
//}

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
    clientS.print(String("{ \"type\":\"searchModule\",") + String(" \"module\":\"lamp\", ") + String(" \"ip\":\"") + ip + "\"}");
    clientS.stop();
  }
}

void setPin() {
  digitalWrite(D1, chanleState[0]);
  digitalWrite(D2, chanleState[1]);
  digitalWrite(D5, chanleState[2]);
  digitalWrite(D6, chanleState[3]);
}

String IpAddress2String(const IPAddress & ipAddress)
{
  return String(ipAddress[0]) + String(".") + \
         String(ipAddress[1]) + String(".") + \
         String(ipAddress[2]) + String(".") + \
         String(ipAddress[3])  ;
}
