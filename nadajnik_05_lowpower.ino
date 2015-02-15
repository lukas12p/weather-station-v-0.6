#include <VirtualWire.h>
#include <Wire.h>
#include <LowPower.h>
#include <dht.h>
dht DHT;
#define DHT21_PIN 5

int wejscie = A0;
byte zasDHT = 6;
byte zas433 = 4;
byte tx = 3;
int led = 13;
int tempA;
int humiA;
int odczyt;
 
void setup() 
{
  vw_set_tx_pin(tx);
  pinMode(wejscie, INPUT);
  pinMode(led, OUTPUT);
  pinMode(zasDHT, OUTPUT);
  pinMode(zas433, OUTPUT); 
  vw_setup(2000);
}
 
void loop() 
{
  delay(100);
  digitalWrite(zasDHT, HIGH);
  delay(50);
  digitalWrite(zas433, HIGH);
  delay(2300);
  vw_setup(1000);
  int chk = DHT.read21(DHT21_PIN);
  delay(1);
  odczyt = analogRead(wejscie);
  delay(2300);
  chk = DHT.read21(DHT21_PIN);
  tempA = 10*DHT.temperature;
  humiA = DHT.humidity; 

  char temp[5];
  dtostrf(tempA, 4, 0, temp);
  char humi[5];
  dtostrf(humiA, 2, 0, humi);
  char v[5];
  dtostrf(odczyt, 4, 0, v);

  strcat(temp, humi);
  strcat(temp, v);
  
  delay(100);
  digitalWrite(led, true);

  vw_send((uint8_t *)temp, strlen(temp));
  vw_wait_tx();
    
  digitalWrite(led, false);
  delay(200);
  digitalWrite(zasDHT, LOW);
  digitalWrite(zas433, LOW);
  
  for (int i = 0; i < 12; i++)
    { 
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    }
}
