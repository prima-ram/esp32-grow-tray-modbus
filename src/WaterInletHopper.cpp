#include "WaterInletHopper.h"
#include "Config.h"
#include "LoadCells.h"
#include "Utils.h"

// Build hopper with built-in valve on I2C address 0x0F (inlet driver).
WaterInletHopper::WaterInletHopper(uint8_t id, Cell *loadCell)
		: id_(id),
			valve1(Valve::Channel::B, 0x0F, Valve::Type::NormallyClosed),
			loadCell_(loadCell)
{
	for (auto &reading : last_readings_)
	{
		reading = -1000;
	}
}

void WaterInletHopper::begin()
{
	valve1.begin();
}

void WaterInletHopper::newReading()
{
	const long reading = loadCell_->Read();

	for (size_t i = 1; i < READINGS_NUMBER; ++i)
	{
		last_readings_[i - 1] = last_readings_[i];
	}
	last_readings_[READINGS_NUMBER - 1] = reading;
}

void WaterInletHopper::loop()
{
	newReading();
	valve1.loop();

	if (!finishingIrrigation_)
	{
		return;
	}

	switch (finishState_)
	{
	case FinishIrrigationState::OpeningV1:
		valve1.open();
		setFinishState(FinishIrrigationState::WaitingEmpty, "WaitingEmpty");
		break;
	case FinishIrrigationState::WaitingEmpty:
		if (!isStableAround(0, Config_Get().tolerance))
		{
			break;
		}
		setFinishState(FinishIrrigationState::ClosingV1, "ClosingV1");
		break;
	case FinishIrrigationState::ClosingV1:
	{
		valve1.close();
		if (LoadCells::instance().requestTare(1, "finish irrigation"))
		{
			setFinishState(FinishIrrigationState::WaitingTare, "WaitingTare");
		}
		break;
	}
	case FinishIrrigationState::WaitingTare:
		if (!LoadCells::instance().isTareInProgress(1))
		{
			finishingIrrigation_ = false;
			setFinishState(FinishIrrigationState::Idle, "Idle");
		}
		break;
	case FinishIrrigationState::Idle:
	default:
		finishingIrrigation_ = false;
		setFinishState(FinishIrrigationState::Idle);
		break;
	}
}

bool WaterInletHopper::readingsAboveMin(long min_value) const
{
	for (const auto reading : last_readings_)
	{
		if (reading < min_value)
		{
			return false;
		}
	}
	return true;
}

bool WaterInletHopper::readingsBelowMax(long max_value) const
{
	for (const auto reading : last_readings_)
	{
		if (reading > max_value)
		{
			return false;
		}
	}
	return true;
}

bool WaterInletHopper::readingsFilled() const
{
	return readingsAboveMin(-200);
}

void WaterInletHopper::printReadings() const
{
	Serial.print("Last readings: ");
	for (size_t i = 0; i < READINGS_NUMBER; ++i)
	{
		Serial.print(last_readings_[i]);
		if (i < READINGS_NUMBER - 1)
		{
			Serial.print(", ");
		}
	}
	Serial.println();
}

bool WaterInletHopper::isStableAround(long target, long tolerance) const
{
	if (!readingsFilled())
	{
		return false;
	}

	long min_val = last_readings_[0];
	long max_val = last_readings_[0];

	for (size_t i = 1; i < READINGS_NUMBER; ++i)
	{
		const long value = last_readings_[i];
		if (value < min_val)
		{
			min_val = value;
		}
		if (value > max_val)
		{
			max_val = value;
		}
	}

	return (min_val >= (target - tolerance)) && (max_val <= (target + tolerance));
}

void WaterInletHopper::setFinishState(FinishIrrigationState state, const char *label)
{
	finishState_ = state;
	if (label != nullptr)
	{
		Serial.println(String("Estado Riego cambiado a: ") + label);
	}
}

void WaterInletHopper::finishIrrigation()
{
	if (finishingIrrigation_)
	{
		return;
	}

	finishingIrrigation_ = true;
	setFinishState(FinishIrrigationState::OpeningV1, "OpeningV1");
}

void WaterInletHopper::openValve1()
{
	valve1.open();
}

void WaterInletHopper::closeValve1()
{
	valve1.close();
}

bool WaterInletHopper::valve1IsOpen() const
{
	return valve1.isOpen();
}

void WaterInletHopper::resetFinishState()
{
	finishingIrrigation_ = false;
	setFinishState(FinishIrrigationState::Idle);
	Serial.println("Estado Riego cambiado a: Idle (reset)");
}

uint8_t WaterInletHopper::irrigationFinishState() const
{
	return finishingIrrigation_ ? 1 : 0;
}
