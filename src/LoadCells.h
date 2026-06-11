#pragma once

#include <Arduino.h>
#include "Cell.h"
#include "Config.h"

class WaterInletHopper;
class DrainWaterHopper;

// Manager for two load cells identified by id (1 and 2)
class LoadCells
{
public:
	static constexpr uint8_t LORAWAN_HISTORY_SIZE = 4;
	static LoadCells &instance();

	// Configure offsets/factors at startup
	void begin();

	// Update both cells
	bool update();

	// Raw HX711 reading by id
	long read(uint8_t sensorId);

	// Per-cell operations
	void setFactor(uint8_t sensorId, float factor);
	void setOffset(uint8_t sensorId, long offset);
	bool requestTare(uint8_t sensorId, const char *source = "unknown");
	bool isTareInProgress(uint8_t sensorId) const;
	float calibrate(uint8_t sensorId, float grams);
	void refreshIfDue(uint32_t intervalMs = 500);
	void processCommand(const String &data,
											WaterInletHopper *waterInlet = nullptr,
											DrainWaterHopper *drain = nullptr);
	void debugLoop();
	Cell &cellFromId(uint8_t sensorId);
	uint8_t recentReadingsCount(uint8_t sensorId) const;
	long recentReading(uint8_t sensorId, uint8_t index) const;

private:
	LoadCells();
	bool isNumber(const String &str);
	void pushReading(uint8_t sensorId, long reading);
	void completePendingTare(uint8_t sensorId);

	struct PendingTare
	{
		bool active = false;
		const char *source = "unknown";
	};

	// Pins of the cells (Grove D0 port -> D0/D1, Grove D1 port -> D2/D3)
	static constexpr uint8_t PIN_DOUT1 = 2;	 // D0 (GPIO1)
	static constexpr uint8_t PIN_SCK1 = 1;	 // D1 (GPIO2)
	static constexpr uint8_t PIN_DOUT2 = 4; // D2 (GPIO3)
	static constexpr uint8_t PIN_SCK2 = 3;	 // D3 (GPIO4)

	unsigned long lastRefreshMs = 0;
	bool refreshInProgress_ = false;
	bool refreshCell1Pending_ = false;
	bool refreshCell2Pending_ = false;
	PendingTare pendingTares_[2];

	Config *cfgPtr = nullptr;
	Cell loadCell1;
	Cell loadCell2;
	long recentReadings_[2][LORAWAN_HISTORY_SIZE];
	uint8_t recentCounts_[2];
	uint8_t recentWriteIndex_[2];
	WaterInletHopper *debugWaterInlet_ = nullptr;
	DrainWaterHopper *debugDrain_ = nullptr;
	enum class DebugPulseState : uint8_t
	{
		Idle = 0,
		V1Open,
		V1Close,
		V2Open,
		V2Close,
		V3Open,
		V3Close
	};
	DebugPulseState debugPulseState_ = DebugPulseState::Idle;
	uint32_t debugPulseNextMs_ = 0;
	uint32_t debugPulseStepMs_ = 2500;
};
