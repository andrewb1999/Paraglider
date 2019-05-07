#include <HardwareSerial.h>

void setup() {
  Serial.begin(9600);
  Serial2.begin(115200, SERIAL_8N1);

}

void loop() {
  char buffer[] = "Hello";
  //Serial.println("David");
  
  if (Serial2.availableForWrite()) {
    Serial.println("David");
    Serial2.write((uint8_t *) buffer, 6);  
  }
  Serial.println("David");
  Serial2.write((uint8_t *) buffer, 6);
  delay(1);

}
