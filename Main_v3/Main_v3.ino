/*
    Main_v3
*/

#include <SPI.h>
#include <Ethernet2.h>
#include <EthernetUdp2.h>
#include <ArduinoJson.h>
#include <LiquidCrystal.h>

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(A8, A9, A10, A11, A12, A13);

// библиотека для работы с GPRS устройством
#include <GPRS_Shield_Arduino.h>
 
// создаём объект класса GPRS и передаём в него объект Serial1 
GPRS gprs(Serial1);

#include <TroykaDHT11.h>

DHT11 dht(A0);


#include <Wire.h>

#include <LPS331.h>

LPS331 ps;

// библиотека для работы с часами реального времени
#include "TroykaRTC.h"

#include <EEPROM.h>

// создаём объект для работы с часами реального времени
RTC clock;

byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 0, 70);


unsigned long previousMillis = 0;
const long interval = 5000;

IPAddress mip(239, 255, 255, 51);
String remote = "";
IPAddress remoteIp(0, 0, 0, 0);

unsigned int localPort = 3235;     //udp resiver

char packetBuffer[UDP_TX_PACKET_MAX_SIZE];

EthernetServer server(4234);     //tcp resiver

EthernetUDP Udp;

EthernetClient clientS;

void setup() {
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("Home_v3");
  lcd.setCursor(0, 1);
  lcd.print("Vasily Yuryev");
  lcd.setCursor(0, 0);
  // открываем последовательный порт для мониторинга действий в программе
  Serial.begin(9600);
  // открываем Serial-соединение с GPRS Shield
  Serial1.begin(9600);
  // ждём, пока не откроется монитор последовательного порта
  // для того, чтобы отследить все события в программе
  while (!Serial) {
  }
  Serial.print("Serial init OK\r\n");
  // включаем GPRS шилд
  gprs.powerOn();
  // проверяем есть ли связь с GPRS устройством
  while (!gprs.init()) {
    // если связи нет, ждём 1 секунду
    // и выводим сообщение об ошибке
    // процесс повторяется в цикле
    // пока не появится ответ от GPRS устройства
    delay(1000);
    Serial.print("Init error\r\n");
  }
  // выводим сообщение об удачной инициализации GPRS Shield
  Serial.println("GPRS init success");

  // start the Ethernet connection:
  //if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // no point in carrying on, so do nothing forevermore:
    // try to congifure using IP address instead of DHCP:
  Ethernet.begin(mac, ip);
  
  Udp.beginMulti(mip, localPort);
  dht.begin();
  Wire.begin();
  if (!ps.init())
  {
    Serial.println("Failed to autodetect pressure sensor!");
    while (1);
  }

  ps.enableDefault();
  clock.begin();
  clock.set(__TIMESTAMP__);
  server.begin();
  delay(1000);
  lcd.clear();
}

void handle_mes() {
  EthernetClient client = server.available();
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
      String teln = root["body"]["teln"].as<String>();
      String mes = root["body"]["mes"].as<String>();
      Serial.println(teln);
      Serial.println(mes);
      char charBufTn[50];
      char charBufMes[200];
      teln.toCharArray(charBufTn, 50);
      mes.toCharArray(charBufMes, 200);
      gprs.sendSMS(charBufTn, charBufMes);
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

    Serial.println("234");
    float pressure = ps.readPressureMillibars();
    float altitude = ps.pressureToAltitudeMeters(pressure);
    float temperature = ps.readTemperatureC();

    Serial.print("p: ");
    Serial.print(pressure);
    Serial.print(" mbar\ta: ");
    Serial.print(altitude);
    Serial.print(" m\tt: ");
    Serial.print(temperature);
    Serial.println(" deg C");
    dht.read();
    // массив для хранения текущего времени
    char timeChar[12] = {0};
    // массив для хранения текущей даты
    char dateChar[12] = {0};
    // массив для хранения текущего дня недели
    char dowChar[12] = {0};

    // запрашиваем данные с часов
    clock.read();
    clock.getTimeStr(timeChar, 12);
    clock.getDateStr(dateChar, 12);
    clock.getDOWStr(dowChar, 12);
    // сохраняем текущее время в массив time
    String timeStr = String(timeChar);
    // сохраняем текущую дату в массив date
    String dateStr = String(dateChar);
    // сохраняем текущий день недели в массив dow
    String dowStr = String(dowChar);

    Serial.print(timeStr);
    Serial.print("  ");
    Serial.print(dateStr);
    Serial.print("  ");
    Serial.println(dowStr);
    int h = dht.getHumidity();
    float tempCorrect = (temperature + dht.getTemperatureC()) / 2;
    Serial.println(tempCorrect);
    lcd.setCursor(0, 0);
    lcd.print(String("Temp=") + String(tempCorrect, 1));
    lcd.setCursor(10, 0);
    lcd.print(String("Hum=") + h);
    lcd.setCursor(8, 1);
    lcd.print(timeStr);
    if (remote != "") {
      String output = "{\"ip\":\"";
      output += "192.168.0.70";
      output += "\",\"type\":\"post\", \"module\":\"main\", \"body\":{ \"temp\":\"";
      output += String(tempCorrect);
      output += "\", \"humid\":\"";
      output += String(h);
      output += "\", \"press\":\"";
      output += String(pressure);
      output += "\", \"time\":\"";
      output += timeStr.substring(3, 5);
      output += "\"}}";

      Serial.println(output);

      clientS.connect(remoteIp, 1337);
      clientS.print(output);
      clientS.stop();
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

    clientS.connect(remoteIp, 1337);
    clientS.println(String("{ \"type\":\"searchModule\",") + String(" \"module\":\"main\", ") + String(" \"ip\":\"192.168.0.70\"}"));
    clientS.stop();
  }
}

String IpAddress2String(const IPAddress& ipAddress)
{
  return String(ipAddress[0]) + String(".") + \
         String(ipAddress[1]) + String(".") + \
         String(ipAddress[2]) + String(".") + \
         String(ipAddress[3])  ;
}
