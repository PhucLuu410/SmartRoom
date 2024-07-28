#define BLYNK_PRINT Serial


#define BLYNK_TEMPLATE_ID "TMPL69pb9xxS_"
#define BLYNK_TEMPLATE_NAME "Blink"
#define BLYNK_AUTH_TOKEN "jUpG4to1mNTrv6BMT7a2GrMzcYdoVs8A"


#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SoftwareSerial.h>


SoftwareSerial mySerial(2, 14);
char ssid[] = "Hoang Phuc";
char pass[] = "0906382269";


#define LED 5 


void setup()
{
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
  pinMode(D7 , OUTPUT);
  pinMode(D6 , INPUT);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  mySerial.begin(115200);
}


BLYNK_WRITE(V4)
{
  digitalWrite(LED, param.asInt());
}


void loop()
{
  Blynk.run();
  if (mySerial.available() > 0)
  {
    String input = mySerial.readString();
    Serial.println(input);   
    int value1 = (input[0] - '0') * 10 + (input[1] - '0');
    int value2 = (input[2] - '0') * 10 + (input[3] - '0');
    Blynk.virtualWrite(V1, value1);
    Blynk.virtualWrite(V2, value2);
  }
  if(digitalRead(D6) == 1)
  {
    digitalWrite(D7, 1);
  }
  Blynk.virtualWrite(V5, D6);
}