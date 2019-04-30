#include <WiFi.h>
#include <WiFiUdp.h>
#include <driver/adc.h>

/* WiFi network name and password */
const char * ssid = "BeagleBone-DE0A";
const char * pwd = "BeagleBone";

// IP address to send UDP data to.
// it can be ip address of the server or 
// a network broadcast address
// here is broadcast address
const char * udpAddress = "192.168.8.1";
const int udpPort = 44444;

//create UDP instance
WiFiUDP udp;

void setup(){
  Serial.begin(9600);
  
  //Connect to the WiFi network
   WiFi.begin(ssid, pwd);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  //This initializes udp and transfer buffer
  udp.begin(udpPort);
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
}

void loop(){
  //data will be sent to server
  char buffer[50];
  char bufferc[50];
  int val;
  float conv;
  //send hello world to server
  val = adc1_get_raw(ADC1_CHANNEL_0);
  conv = (val+1.0)/(4096.0);
  sprintf(buffer, "%1.4f    ", conv);
  udp.beginPacket(udpAddress, udpPort);
  udp.write((uint8_t *)buffer, 11);
  udp.endPacket();
  sprintf(bufferc, "%s", buffer);
  memset((uint8_t *)buffer, 0, 50);
  //processing incoming packet, must be called before reading the buffer
  udp.parsePacket();
  //receive response from server, it will be HELLO WORLD
  if(udp.read((uint8_t *)buffer, 50) > 0){
    Serial.print("Server to client: ");
    Serial.println((char *)buffer);
    Serial.println(bufferc);
  }
  //Wait for 1 second
  delay(10);
}
