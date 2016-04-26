#include "DipConfig.h"
#include <SPI.h>

#define LED_DMX 9
#define ENABLE_RX 2

#define STEP 4
#define DIR 5

DipConfig dip(10);

void setup(){
  Serial.begin(9600);
  
  pinMode(LED_DMX, OUTPUT);
  
  pinMode(ENABLE_RX, OUTPUT);
  digitalWrite(ENABLE_RX, LOW);
  
  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
}

void loop() {
  uint16_t t = dip.get();
  Serial.println(t, BIN);
  digitalWrite(LED_DMX, HIGH);
  digitalWrite(STEP, HIGH);
  digitalWrite(DIR, LOW);
  delay(100);
  digitalWrite(LED_DMX, LOW);
  digitalWrite(STEP, LOW);
  digitalWrite(DIR, HIGH);
  delay(100);
}
