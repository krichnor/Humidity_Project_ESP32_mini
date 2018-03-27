//////////  Sensor NAME  //////////
// For my Raspberry Pi
//const char* SensorName     = "krich/garden01";
// For Ubidots
const char* SensorName     = "/v1.6/devices/5ab1e966c03f9733ab070024";

#define hue_sensor_pin 36  // ADC00
#define hue_ref_pin 39//SVN
static char SoilHumid[8];
static char SoilAnalogs[8];
static char UpTime[10];
static char testchar[8];

////////// init variables //////////
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
// For my raspberry PI
//const char* mqttServer = "192.168.1.38";
//const int mqttPort = 1883;
//const char* mqttUser = "ESP32";
//const char* mqttPassword = "khunkrich";
// For Ubidots 
const char* mqttServer = "things.ubidots.com";
const int mqttPort = 1883;
const char* mqttUser = "A1E-c0gPMPmIlP9VC0KBj4OkQu8LVexXHu";
const char* mqttPassword = "";

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
  Serial.print("Hello message : ");
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
  Serial.begin(115200);
  
  analogReadResolution(13);
  delay(1000);
  analogRead(hue_sensor_pin);
  
  ////////// WIFI chacking and setting up ////////////
  if (WiFi.status() != WL_CONNECTED) {
    WIFIconnect();
  }
  if (WiFi.status() == WL_CONNECTED) {
    MQTTconnect();    //MQTT setup
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

  // Soil Humidity part
  double output_value = analogRead(hue_sensor_pin);
  double ref_read = map(analogRead(hue_ref_pin),5730,0,5730,8190);
  
  float output_value_map= mapfloat(output_value,ref_read,0,0,100); //Map with refference voltage
  
  double analog_value = output_value;
  
  //float output_value_map = mapfloat(output_value,3280,160,0,100);   // for ESP32
  //float output_value_map = mapfloat(output_value,2500,160,0,100);   // for LOLIN32-OLED
  
  
//  if(output_value_map >100){
//    output_value_map =100;
//  }
  Serial.print("Mositure : ");
  Serial.print(output_value_map);
  Serial.println("%");
  Serial.print("Analog value = ");
  Serial.println(analog_value);
  //sprintf(SoilHumid,"%f",output_value);
  dtostrf(analog_value,6,2,SoilAnalogs);
  dtostrf(output_value_map,6,2,SoilHumid);


  Serial.print("Analog Ref = ");
  Serial.println(ref_read);


  
  Publish_Soil_Humidity();

  
  delay(10000); // 10 sec interval loop
}
/*---------------------------- SUB FUNCTION ----------------------------*/


void WIFIconnect()
{
  int mytimeout = millis() / 1000;
  WiFi.begin(ssid, password);
  Serial.println();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("Connecting to WiFi...");
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
      sprintf (MQtopic, "%s/SoilHumid", SensorName);
      client.publish(MQtopic,SoilHumid);
      sprintf(MQtopic,"%s/SoilAnalog", SensorName);
      client.publish(MQtopic,SoilAnalogs);
      
      dtostrf(millis()/1000,8,2,UpTime);
      sprintf(MQtopic,"%s/UpTime_r", SensorName);
      client.publish(MQtopic,UpTime,true);
      
      sprintf(MQtopic,"%s/UpTime", SensorName);
      client.publish(MQtopic,UpTime);
      Serial.print("Uptime = ");
      Serial.println(UpTime);
      
      client.subscribe("/v1.6/devices/5ab1e966c03f9733ab070024/getmsg");
      
    } else {
        Serial.println("Connecting to MQTT");
        Serial.print("failed with state ");
        Serial.print(client.state());
    }
  }
}

float mapfloat(long x, long in_min, long in_max, long out_min, long out_max)
{
 return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}


