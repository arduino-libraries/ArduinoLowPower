#include <dht.h>

#define dataPin 2
dht DHT; 

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);
}

void loop() {

  digitalWrite(LED_BUILTIN,HIGH);
  int readData = DHT.read11(dataPin); // DHT11

  float t = DHT.temperature; 
  float h = DHT.humidity; 
 
  Serial.print("Temperature = ");
  Serial.print(t);
  Serial.print(" C | ");

  Serial.print("Humidity = ");
  Serial.print(h);
  Serial.println(" % ");

  delay(2000);

}
