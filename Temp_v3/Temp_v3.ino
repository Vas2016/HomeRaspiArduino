/*
    Temp_v3
*/

// Include the libraries we need
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

#include <ESP8266WiFi.h>
#include <WiFiUDP.h>


#include <ArduinoJson.h>


const char* ssid = "DIR-300A";
const char* password = "8848evev";

unsigned long previousMillis = 0;
const long interval = 8000;

IPAddress mip(239, 255, 255, 50);
String remote = "";
IPAddress remoteIp;

unsigned int localPort = 3234;     //udp resiver

char packetBuffer[UDP_TX_PACKET_MAX_SIZE];

WiFiServer server(4234);     //tcp resiver

WiFiUDP Udp;

WiFiClient clientS;

String ip = "";


void setup() {
  Serial.begin(115200);
  delay(10);
  sensors.begin();
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
}

//void handle_mes() {
//  WiFiClient client = server.available();
//  if (client) {
//    Serial.println("new req");
//
//    String req = client.readStringUntil('\r');
//    Serial.println(req);
//    /*
//      {"ip":"192.168.0.51", "module":"lamp", "type":"comande", "body":{"ch0":"0", "ch1":"1", "ch2":"0", "ch3":"0"}}
//    */
//    DynamicJsonBuffer jsonBuffer;
//
//    JsonObject& root = jsonBuffer.parseObject(req);
//
//    Serial.println("its my mes");
//    String _type = "";
//    _type = root["type"].as<String>();
//    if (_type == "comande") {
//      Serial.println("comande");
//      chanleState[0] = root["body"]["ch0"].as<String>().toInt();
//      chanleState[1] = root["body"]["ch1"].as<String>().toInt();
//      chanleState[2] = root["body"]["ch2"].as<String>().toInt();
//      chanleState[3] = root["body"]["ch3"].as<String>().toInt();
//      Serial.println(chanleState[0]);
//      Serial.println(chanleState[1]);
//      Serial.println(chanleState[2]);
//      Serial.println(chanleState[3]);
//      setPin();
//    } else if (_type == "comande_ir") {
//      Serial.println("comande_ir");
//      String code2 = root["body"]["ir"].as<String>();
//      unsigned long code = strtoul(code2.c_str(), NULL, 10);
//      Serial.println(code2);
//      Serial.println(code, DEC);
//      irsend.sendNEC(code, 32);
//    } else {
//
//    }
//    client.stop();
//  }
//}

void loop() {
  handle_search();
  //handle_mes();
  handle_sendChData();
}

void handle_sendChData() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    if (remote != "") {
      Serial.println("234");
      DynamicJsonBuffer jsonBuffer;
      JsonObject& mes = jsonBuffer.createObject();
      DynamicJsonBuffer jsonBuffer2;
      JsonObject& body = jsonBuffer2.createObject();
      mes["ip"] = ip;
      mes["type"] = "post";
      mes["module"] = "temp";
      
      Serial.print("Requesting temperatures...");
      sensors.requestTemperatures();
      Serial.println("DONE");
      Serial.print("Temperature for the device 0 (index 0) is: ");
      float temp = sensors.getTempCByIndex(0);
      Serial.println(temp);
      body["temp"] = String(temp);

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
    clientS.print(String("{ \"type\":\"searchModule\",") + String(" \"module\":\"temp\", ") + String(" \"ip\":\"") + ip + "\"}");
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
