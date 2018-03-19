//////////  Sensor NAME  //////////
const char* SensorName     = "krich/garden01";

#define hue_sensor_pin 36
int output_value ;
static char SoilHumid[7];
static char SoilAnalog[7];
static char UpTime[7];

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
  Serial.begin(115200);
  
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
  output_value= analogRead(hue_sensor_pin);
  output_value = map(output_value,1620,160,0,100);
  double analog_value = analogRead(hue_sensor_pin);
  if(output_value >100){
    output_value =100;
  }
  Serial.print("Mositure : ");
  Serial.print(output_value);
  Serial.println("%");
  Serial.print("Analog value = ");
  Serial.println(analog_value);
  dtostrf(output_value,6,2,SoilHumid);
  dtostrf(analog_value,6,2,SoilAnalog);
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
      client.publish(MQtopic,SoilAnalog);
      
      dtostrf(millis()/1000,6,2,UpTime);
      sprintf(MQtopic,"%s/UpTime_r", SensorName);
      client.publish(MQtopic,UpTime,true);
      
      sprintf(MQtopic,"%s/UpTime", SensorName);
      client.publish(MQtopic,UpTime);
      

      
    } else {
        Serial.println("Connecting to MQTT");
        Serial.print("failed with state ");
        Serial.print(client.state());
    }
  }
}
