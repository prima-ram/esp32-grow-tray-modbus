#pragma once
#include "Arduino.h"

String createStatusTopic(uint8_t sensorId);
String createReadingTopic(uint8_t sensorId, String measure);
String createTareTopic(uint8_t sensorId);