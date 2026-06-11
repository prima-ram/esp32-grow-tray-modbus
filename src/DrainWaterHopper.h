#pragma once

#include "Arduino.h"
#include "Valve.h"
#include "Cell.h"

// Drain hopper with two valves.
class DrainWaterHopper
{
private:
	static const uint8_t READINGS_NUMBER = 10;
	long last_readings_[READINGS_NUMBER];
	uint8_t id_;
	Valve valve1_;
	Valve valve2_;
	Cell *loadCell_ = nullptr;
	bool finishingDrain_ = false;

	bool readingsSimilar(int tolerance = 50) const;
	bool readingsAboveMin(long min_value) const;
	bool readingsAboveMax(long max_value) const;
	bool readingsFilled() const;
	void newReading();

	enum class FinishDrainState : uint8_t
	{
		Idle = 0,
		ClosingV3,
		OpeningV2,
		WaitingEmpty,
		ClosingV2,
		WaitingTare,
		OpeningV3
	};

	FinishDrainState finishState_ = FinishDrainState::Idle;
	void setFinishState(FinishDrainState state, const char *label = nullptr);

public:
	explicit DrainWaterHopper(uint8_t id, Cell *loadCell = nullptr);
	void begin();
	void loop();
	void openValve2();
	void closeValve2();
	void openValve3();
	void closeValve3();
	bool valve2IsOpen() const;
	bool valve3IsOpen() const;
	void printReadings() const;
	bool isStableAround(long target, long tolerance) const;
	void finishDrain();
	void resetFinishState();
	uint8_t drainFinishState() const;
};
