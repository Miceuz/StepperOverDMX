#include "DipConfig.h"
#include <SPI.h>
#include "TimerOne.h"
#include "Conceptinetics.h"
#include "FilterCascade.h"

#define LED_DMX 8
#define ENABLE_RX 2

#define STEP 9
#define DIR 4

DipConfig dip(10);

#define SLAVE_CHANNELS 2

void OnFrameReceiveComplete (void);

DMX_Slave dmxSlave(SLAVE_CHANNELS, ENABLE_RX);

uint32_t lastDMXFrameTimestamp = 0;
uint8_t h[] = {2, 1, 4};
FilterCascade filter(1, h); 
uint16_t dmxAddress = 0;

void setup(){
  pinMode(LED_DMX, OUTPUT);
  
  if(dip.isTestMode()){
    while(true) {
      digitalWrite(LED_DMX, HIGH);
      delay(50);
      digitalWrite(LED_DMX, LOW);
      delay(50);
    }
  } else {
    dmxSlave.onReceiveComplete ( OnFrameReceiveComplete );
    dmxSlave.enable ();
    dmxAddress = dip.get();
    dmxSlave.setStartAddress (dmxAddress);
  }

  pinMode(STEP, OUTPUT);
  pinMode(DIR, OUTPUT);
  digitalWrite(DIR, LOW);
  Timer1.initialize(4000);  // 4ms = 250 Hz
  //Timer1.pwm(STEP, 512);
  Timer1.stop();
}

#define HOUSEKEEPING_INTERVAL_MS 100
#define DMX_NO_FRAME_INTERVAL_MS 400
#define MAX_PPS (130 * 15)
#define MIN_TIMER_PERIOD_US 1000000 / MAX_PPS 
#define ACCELERATION_INTERVAL_MS 100

uint32_t housekeepingTimestamp = 0;
static inline uint8_t isHousekeepingTime() {
  return (millis() - housekeepingTimestamp) > HOUSEKEEPING_INTERVAL_MS;
}

static inline uint8_t isDmxDataAvailable() {
  return (millis() - lastDMXFrameTimestamp) < DMX_NO_FRAME_INTERVAL_MS;
}

uint32_t accelerationTimestamp = 0;
static inline uint8_t isAccelerationTime() {
  return (millis() - accelerationTimestamp) > ACCELERATION_INTERVAL_MS;
}

int32_t targetPPS = 0;
int32_t currPPS = 0;

uint16_t acceleration = 10;

uint8_t targetDir = 0;
uint8_t currDir = 0;

static inline void updateDir() {
  if(currDir) {
    digitalWrite(DIR, HIGH);
  } else {
    digitalWrite(DIR, LOW);
  }
}

static inline void slowDownAndChangeDir() {
  if(currPPS > 0) {
    if(currPPS > acceleration) {//sito clusterfucko reikia, kad nors karta pilnai sustabdytu
      currPPS = currPPS - acceleration;
    } else {
      currPPS = 0;
    }
  } else {
    currDir = targetDir;
    updateDir();
  } 
}

static inline void updateSpeed() {
  if(targetDir != currDir) {
    slowDownAndChangeDir();
  } else {
    if(abs(currPPS - targetPPS) < acceleration) {
      currPPS = targetPPS;
    } else if(currPPS < targetPPS) {
      currPPS = currPPS + acceleration; 
    } else if(currPPS > targetPPS) {
      if(currPPS > acceleration) {
        currPPS = currPPS - acceleration;
      } else {
        currPPS = 0;
      }
    }
    
  }
  

  if(currPPS == 0) {
    Timer1.stop();
  } else {
    updateDir();
    uint32_t period = 1000000 / currPPS;
    Timer1.setPeriod(period);
    Timer1.pwm(STEP, 512);  
  }
  accelerationTimestamp = millis();
}

void loop() {
  if(isDmxDataAvailable()) {
      digitalWrite(LED_DMX, HIGH);
  } else {
      digitalWrite(LED_DMX, LOW);
      Timer1.stop();
  } 

  if(isHousekeepingTime()) {
    if(dip.get() != dmxAddress && !dip.isTestMode()) {
      dmxAddress = dip.get();
      dmxSlave.setStartAddress(dmxAddress);
    }

    acceleration = map(analogRead(A7), 0, 1023, 5, 30);
  }

  if(isAccelerationTime()){
    updateSpeed();
  }
}

void OnFrameReceiveComplete (void) {
  lastDMXFrameTimestamp = millis();
  uint8_t dmxVal = dmxSlave.getChannelValue(1);
  targetPPS = map(dmxVal, 0, 255, 0, MAX_PPS);
  dmxVal = dmxSlave.getChannelValue(2);
  if(dmxVal > 127) {
    targetDir = 1;
  } else {
    targetDir = 0;
  }
}


