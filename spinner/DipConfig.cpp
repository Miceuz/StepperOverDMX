#include "DipConfig.h"
#include <SPI.h>
#include <Arduino.h>

DipConfig::DipConfig(uint8_t _latchPin):
	latchPin(_latchPin)
{
	pinMode(latchPin, OUTPUT);
        SPI.begin();
}

uint16_t DipConfig::get() {
	digitalWrite(latchPin, LOW);
	delayMicroseconds(5);
	digitalWrite(latchPin, HIGH);

        uint8_t lower = SPI.transfer(0);
        uint8_t higher = SPI.transfer(0);

	return (higher << 8) | lower;
}

