#include "SSD1306.h" 
#include <DHT.h>      // Library DHT-sensor-library-master.zip

//////////  Sensor NAME  //////////
const char* SensorName     = "krich";

#define hue_sensor_pin 36
int output_value ;


//////////  DHT Setting // for Temperature sensor type DHT11
#define DHTTYPE DHT11 // Library DHT-sensor-library-master.zip
#define DHTPIN 2     // Library DHT-sensor-library-master.zip
DHT dht(DHTPIN, DHTTYPE);

// Temporary variables // for Temperature sensor type DHT11
static char celsiusTemp[7];
static char humidityTemp[7];
static char HIndTemp[7];
static char SoilHumid[7];
char charSensorDisplay[30];   //Buffer to display message

int sensorPin = 36;   // select the input pin for DHT11
int sensorValue = 0;  // variable to store the value coming from the sensor
void TempSensorReadDisplay();

////////// KBTG Greeting init variables //////////
int Device_state = -2;   //1st WifiConnect= 0, reconWIFI >1
int Device_state_prev = 0;


////////// WIFI & MQTT Header //////////
#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid     = "true_home2G_751";
const char* password = "17a53751";
String Statuses[] =  { "WL_IDLE_STATUS(0)", "WL_NO_SSID_AVAIL(1)", "WL_SCAN_COMPLETED(2)", "WL_CONNECTED(3)", "WL_CONNECT_FAILED(4)", "WL_CONNECTION_LOST(5)", "WL_DISCONNECTED(6)"};

WiFiClient espClient;
void WIFIconnect();

////////// MQTT Setting //////////
const char* mqttServer = "192.168.1.40";
const int mqttPort = 1883;
const char* mqttUser = "ESP32";
const char* mqttPassword = "khunkrich";
PubSubClient client(espClient);
String getMessage;
char MQtopic[50];

void MQTTconnect();
void callback(char* topic, byte* payload, unsigned int length) {
  getMessage = "";
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    getMessage = getMessage + (char)payload[i];
  }
  Serial.println(getMessage);
}

////////// NTP Setting //////////
#include <TimeLib.h>  // need PaulStoffregen library and Time-master.zip
#include <WiFiUdp.h>

static const char ntpServerName[] = "us.pool.ntp.org";
const int timeZone = 7;     // Central European Time (Bangkok GMT +7)
unsigned int localPort = 8888;  // local port to listen for UDP packets
const int NTP_PACKET_SIZE = 48; // NTP time is in the first 48 bytes of message
WiFiUDP ntpUDP;
time_t prevDisplay = 0; // when the digital clock was displayed
void sendNTPpacket(IPAddress &address);
char charDateTime[25];
byte packetBuffer[NTP_PACKET_SIZE]; //buffer to hold incoming & outgoing packets


void setup() {
  dht.begin();
  Serial.begin(115200);
  
  ////////// WIFI chacking and setting up ////////////
  if (WiFi.status() != WL_CONNECTED) {
    WIFIconnect();
  }
  if (WiFi.status() == WL_CONNECTED) {
    MQTTconnect();    //MQTT setup
    ////////// NTP chacking and setting up ////////////
    if (timeStatus() != timeSet) {
      Serial.print("Start NTP Setup Process");
      setSyncProvider(getNtpTime);
      setSyncInterval(300);
      if (timeStatus() == timeSet) { //update the display only if time has changed
        Serial.print("NTP Time is set -> ");
        sprintf (charDateTime, "%02d:%02d:%02d %02d/%02d/%04d", hour(), minute(), second(), day(), month(), year());
        Serial.println(charDateTime);
      }
    }
  }

  Serial.println("Reading From the Soil Humidity Sensor ...");
}

void loop() {

  // WiFi connect checking status //
  if (WiFi.status() != WL_CONNECTED) {
    setup();
  }

                
 
  
  Serial.print("Device_state = ");
  Serial.println(Device_state);

  Serial.print("Device_state_prev = ");
  Serial.println(Device_state_prev);

  ////////// Show NTS if not set ////////////
  if (timeStatus() == timeNotSet) {
    Serial.println("NTP Time is not set");
  }
  /*  ////////// Show Date -  ////////////
    if (timeStatus() != timeNotSet) {
      if (now() != prevDisplay) { //update the display only if time has changed
        prevDisplay = now();
        Serial.print("Time stamp from NTP -> ");
        digitalClockDisplay();
        Serial.println();
      }
    }   */

  
    

  // Soil Humidity part
  output_value= analogRead(hue_sensor_pin);
  output_value = map(output_value,1620,160,0,100);
  if(output_value >100){
    output_value =100;
  }
  Serial.print("Mositure : ");
  Serial.print(output_value);
  Serial.println("%");
  Serial.print("Analog value = ");
  Serial.println(analogRead(hue_sensor_pin));
  dtostrf(output_value,6,2,SoilHumid);
  // Soil Humidity part.

  Publish_Soil_Humidity();

  delay(10000); // 10 sec interval loop
}
/*---------------------------- SUB FUNCTION ----------------------------*/


void WIFIconnect()
{
  int mytimeout = millis() / 1000;
  WiFi.begin(ssid, password);
  Serial.println();
  Serial.print("Connecting to WiFi..");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi...");
    Serial.print(".");
    if ((millis() / 1000) > mytimeout + 4) { // try for less than 4 seconds to connect to WiFi router
      break;
    }
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("SSID: ");
    Serial.print(String(ssid));
    Serial.println(" Cannot be connected.");
    Serial.print("  WiFi.status = ");
    Serial.println(Statuses[WiFi.status()]);  //Show WIFI connection status error
  } else {
    Serial.println("Connected to the WiFi network");
    Serial.println("Connected to WiFi: Done");
    Device_state ++;
  }
}

void MQTTconnect() {
  ///////// MQTT setup /////////
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP32Client", mqttUser, mqttPassword )) {
      Serial.println(" Connected to MQTT: Done");
      Serial.println("KBTG IoT:");
      Serial.println("Temperature Sensor Proj.");
      Serial.println("Connected MQTT: Done");
      Serial.println("SSID: " + String(ssid));
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      Serial.println("KBTG IoT:");
      Serial.println("Temperature Sensor Proj.");
      Serial.println("Connecting to MQTT");
      Serial.println("SSID: " + String(ssid));
      delay(1000);  //Re-connecting in a minute
    }
  }
  client.disconnect();
}


 
void Publish_Soil_Humidity() {
  ////////// Publish MQTT data ////////////
  if (WiFi.status() == WL_CONNECTED) {
    if (client.connect("ESP32Client", mqttUser, mqttPassword )) {
      sprintf (charDateTime, "%02d:%02d:%02d %02d/%02d/%04d", hour(), minute(), second(), day(), month(), year());
      //sprintf (MQtopic, "%s/Temp", SensorName);
      //client.publish(MQtopic, celsiusTemp);
      //sprintf (MQtopic, "%s/Hue", SensorName);
      //client.publish(MQtopic, humidityTemp);
      //sprintf (MQtopic, "%s/HInd", SensorName);
      //client.publish(MQtopic, HIndTemp);
      sprintf (MQtopic, "%s/SoilHumid", SensorName);
     // dtostrf(hic, 6, 2, HIndTemp);
      client.publish(MQtopic,SoilHumid);
      if (Device_state < 1) {
        sprintf (MQtopic, "%s/TimeBoot", SensorName);
        client.publish(MQtopic, charDateTime);
        Device_state ++;
        Device_state_prev = Device_state;
        Serial.print("After stamp boot time, change Device_state to ");
        Serial.println(Device_state);
      } else if (Device_state != Device_state_prev) {
        sprintf (MQtopic, "%s/TimeReconWIFI", SensorName);
        client.publish(MQtopic, charDateTime);
        sprintf (charDateTime, "%i", Device_state);
        sprintf (MQtopic, "%s/TimeReconTime", SensorName);
        client.publish(MQtopic, charDateTime);
        Device_state_prev = Device_state;
      } else {
        sprintf (MQtopic, "%s/TimeStamp", SensorName);
        client.publish(MQtopic, charDateTime);
        client.disconnect();
      }
    } else {
        Serial.println("Connecting to MQTT");
        Serial.print("failed with state ");
        Serial.print(client.state());
    }
  }
}


/*-------- NTP code ----------*/
time_t getNtpTime()
{
  IPAddress ntpServerIP; // NTP server's ip address

  while (ntpUDP.parsePacket() > 0) ; // discard any previously received packets
  Serial.println("Transmit NTP Request");
  // get a random server from the pool
  WiFi.hostByName(ntpServerName, ntpServerIP);
  Serial.print(ntpServerName);
  Serial.print(": ");
  Serial.println(ntpServerIP);
  sendNTPpacket(ntpServerIP);
  uint32_t beginWait = millis();
  while (millis() - beginWait < 1500) {
    int size = ntpUDP.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      Serial.println("Receive NTP Response");
      ntpUDP.read(packetBuffer, NTP_PACKET_SIZE);  // read packet into the buffer
      unsigned long secsSince1900;
      // convert four bytes starting at location 40 to a long integer
      secsSince1900 =  (unsigned long)packetBuffer[40] << 24;
      secsSince1900 |= (unsigned long)packetBuffer[41] << 16;
      secsSince1900 |= (unsigned long)packetBuffer[42] << 8;
      secsSince1900 |= (unsigned long)packetBuffer[43];
      return secsSince1900 - 2208988800UL + timeZone * SECS_PER_HOUR;
    }
  }
  Serial.println("No NTP Response :-(");
  return 0; // return 0 if unable to get the time
}

// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress &address)
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;
  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  ntpUDP.beginPacket(address, 123); //NTP requests are to port 123
  ntpUDP.write(packetBuffer, NTP_PACKET_SIZE);
  ntpUDP.endPacket();
}
