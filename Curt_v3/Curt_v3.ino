/*
    Curt_v3
*/

#include <ESP8266WiFi.h>
#include <WiFiUDP.h>

#include <ArduinoJson.h>
#include <Servo.h> 
 
Servo myservo;

const char* ssid = "DIR-300A";
const char* password = "8848evev";

//unsigned long previousMillis = 0;
//const long interval = 1000;

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
  myservo.attach(3); 
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

void handle_mes() {
  WiFiClient client = server.available();
  if (client) {
    Serial.println("new req");

    String req = client.readStringUntil('\r');
    Serial.println(req);
    /*
      {"ip":"192.168.0.51", "module":"curt", "type":"comande", "body":{"servo":"0", "ch1":"1", "ch2":"0", "ch3":"0"}}
    */
    DynamicJsonBuffer jsonBuffer;

    JsonObject& root = jsonBuffer.parseObject(req);

    Serial.println("its my mes");
    String _type = "";
    _type = root["type"].as<String>();
    if (_type == "comande") {
      Serial.println("comande");
      if(root["body"]["servo"].as<String>().toInt() == 1){
        myservo.write(180);
      }
      else{
        myservo.write(0);
      }
    }
    client.stop();
  }
}

void loop() {
  handle_search();
  handle_mes();
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
    clientS.print(String("{ \"type\":\"searchModule\",") + String(" \"module\":\"curt\", ") + String(" \"ip\":\"") + ip + "\"}");
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
