#pragma once
#include <Arduino.h>

struct Config
{
	String mac;
	uint32_t magic;
	long offset1;
	float factor1;
	long offset2;
	float factor2;
	uint8_t tolerance;
};

// Magic value used to check whether the flash config is valid
constexpr uint32_t CONFIG_MAGIC = 0xCBFEBCBB;

// Initialize/load configuration (from NVS/EEPROM/FS)
void Config_Load();

// Access the global configuration object
Config &Config_Get();

// (Optional) save changes
void Config_Save();
