#pragma once

#include "Arduino.h"
#include "Valve.h"
#include "Cell.h"

class WaterInletHopper
{
private:
	static const uint8_t READINGS_NUMBER = 10;
	long last_readings_[READINGS_NUMBER];
	uint8_t id_;
	Valve valve1;
	Cell *loadCell_ = nullptr;
	bool zeroTareDone_ = false;

	// Check that all readings are above a minimum.
	bool readingsAboveMin(long min_value) const;
	// Check that all readings are below a maximum.
	bool readingsBelowMax(long max_value) const;
	// True when the ring buffer is filled with valid readings.
	bool readingsFilled() const;
	bool finishingIrrigation_ = false;
	enum class FinishIrrigationState : uint8_t
	{
		Idle = 0,
		OpeningV1,
		WaitingEmpty,
		ClosingV1,
		WaitingTare
	};
	FinishIrrigationState finishState_ = FinishIrrigationState::Idle;
	// Check if readings are stable around a target.
	bool isStableAround(long target, long tolerance) const;
	// Push a new reading and update valve state.
	void newReading();
	void setFinishState(FinishIrrigationState state, const char *label = nullptr);

public:
	// Create a hopper
	explicit WaterInletHopper(uint8_t id, Cell *loadCell);
	// Initialize hopper resources (e.g., valve).
	void begin();
	// Periodic logic check (call from main loop).
	void loop();
	// Manual valve control.
	void openValve1();
	void closeValve1();
	bool valve1IsOpen() const;
	// Print last readings for debugging.
	void printReadings() const;
	// Run irrigation-finish logic.
	void finishIrrigation();
	// Force finish-irrigation state back to Idle.
	void resetFinishState();
	// Current irrigation-finish state: 0=Idle, 1=Finishing.
	uint8_t irrigationFinishState() const;
};
