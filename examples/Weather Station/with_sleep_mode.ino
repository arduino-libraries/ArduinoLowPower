/* This example contains the arduino code for 
weather station with sleep mode clearly 
depicting the use case of LowPower library.
Go through the without_sleep_mode.ino code 
for a better understanding of it's working.*/

#include <dht.h>
#include <LowPower.h>

#define dataPin 2
dht DHT; 

void setup()
{
  Serial.begin(9600);
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);
}

void loop()
{

  Serial.println("Get Data From DHT11");
  delay(1000);
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
  Serial.println("Arduino:- I am going for a Nap");
  delay(200);
  digitalWrite(LED_BUILTIN,LOW);
  LowPower.idle(SLEEP_8S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, 
                 SPI_OFF, USART0_OFF, TWI_OFF);
                 
  Serial.println("Arduino:- Hey I just Woke up");
  Serial.println("");
  delay(2000);
  
}
