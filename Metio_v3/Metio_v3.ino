/*
    Temp_v3
*/
#include <Wire.h>
#include <Adafruit_BME280.h>

Adafruit_BME280 bme;

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>


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
  bool status;
  Wire.begin(0, 2);
  // default settings
  status = bme.begin();
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  Serial.println("-- Default Test --");
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
      {"ip":"192.168.0.51", "module":"lamp", "type":"comande", "body":{"ch0":"0", "ch1":"1", "ch2":"0", "ch3":"0"}}
    */
    DynamicJsonBuffer jsonBuffer;

    JsonObject& root = jsonBuffer.parseObject(req);

    Serial.println("its my mes");
    String _type = "";
    _type = root["type"].as<String>();
    if (_type == "get") {
      if (remote != "") {
        Serial.println("get");
        DynamicJsonBuffer  jsonBuffer;
        JsonObject& mes  = jsonBuffer.createObject();
        DynamicJsonBuffer  jsonBuffer2;
        JsonObject& body = jsonBuffer2.createObject();
        mes["ip"]     =  ip;
        mes["type"]   = "post";
        mes["module"] = "metio";

        float temp    = bme.readTemperature();
        float humi    = bme.readHumidity   ();
        float pres    = bme.readPressure   () / 100.0F;

        Serial.println(String(pres));

        body["temp"]  = String(temp);
        body["humi"]  = String(humi);
        body["pres"]  = String(pres);

        mes["body"]   = body;

        String output;
        mes.printTo(output);
        Serial.println(output);

        clientS.connect(remoteIp, 1337);
        clientS.print(output);
        clientS.stop();
        //      sendChData = false;
      }
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
//
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
    clientS.print(String("{ \"type\":\"searchModule\",") + String(" \"module\":\"metio\", ") + String(" \"ip\":\"") + ip + "\"}");
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
