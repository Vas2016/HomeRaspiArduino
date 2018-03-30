/*
    Plant_v3
*/

#include <ESP8266WiFi.h>
#include <WiFiUDP.h>

#include <TroykaDHT11.h>

DHT11 dht(D2);

#include <ArduinoJson.h>

const char* ssid = "DIR-300A";
const char* password = "8848evev";

unsigned long previousMillis = 0;
const long interval = 5000;

IPAddress mip(239, 255, 255, 50);
String remote = "";
IPAddress remoteIp;

unsigned int localPort = 3234;     //udp resiver

char packetBuffer[UDP_TX_PACKET_MAX_SIZE];

WiFiServer server(4234);     //tcp resiver

WiFiUDP Udp;

WiFiClient clientS;

String ip = "";

int humidM = 0;

boolean tagleP1 = false;
boolean tagleP2 = false;

int chanleState[] = {0, 0, 0};

void setup() {
  pinMode(D1, OUTPUT);
  pinMode(D5, OUTPUT);
  pinMode(D6, OUTPUT);
  pinMode(D7, OUTPUT);
  Serial.begin(115200);
  delay(10);


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
  dht.begin();
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

    DynamicJsonBuffer jsonBuffer;

    JsonObject& root = jsonBuffer.parseObject(req);

    Serial.println("its my mes");
    String _type = "";
    _type = root["type"].as<String>();
    if (_type == "comande_pompa") {
      Serial.println("comande_pompa");
      tagleP1 = true;

    } else if (_type == "comande") {
      Serial.println("comande_rgb");
      chanleState[0] = root["body"]["r"].as<String>().toInt();
      chanleState[1] = root["body"]["g"].as<String>().toInt();
      chanleState[2] = root["body"]["b"].as<String>().toInt();
      setPin();
    }
    else {

    }
    client.stop();
  }
}

void loop() {
  handle_search();
  handle_mes();
  handle_sendChData();
}

void handle_sendChData() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    dht.read();
    humidM = analogRead(0);
    String output = "{\"ip\":\"";
    output += ip;
    output += "\",\"type\":\"post\", \"module\":\"plant\", \"body\":{ \"temp\":\"";
    output += String(dht.getTemperatureC());
    output += "\", \"humid\":\"";
    output += String(dht.getHumidity());
    output += "\", \"humidM\":\"";
    output += String(humidM);

    output += "\"}}";
    if (remote != "") {
      Serial.println(output);

      clientS.connect(remoteIp, 1337);
      clientS.print(output);
      clientS.stop();
    }
    if (tagleP2) {
      tagleP1 = false;
      digitalWrite(D1, LOW);
      tagleP2 = false;
    }

    if (humidM < 250) {
      tagleP1 = true;
    }
//    if (humidM > 750) {
//      tagleP2 = true;
//      tagleP1 = false;
//    }
    if (tagleP1) {
      digitalWrite(D1, HIGH);
      tagleP2 = true;
      tagleP1 = false;
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
    clientS.print(String("{ \"type\":\"searchModule\",") + String(" \"module\":\"plant\", ") + String(" \"ip\":\"") + ip + "\"}");
    clientS.stop();
  }
}

void setPin() {
  analogWrite(D5, chanleState[0]);
  analogWrite(D6, chanleState[1]);
  analogWrite(D7, chanleState[2]);

}

String IpAddress2String(const IPAddress & ipAddress)
{
  return String(ipAddress[0]) + String(".") + \
         String(ipAddress[1]) + String(".") + \
         String(ipAddress[2]) + String(".") + \
         String(ipAddress[3])  ;
}
