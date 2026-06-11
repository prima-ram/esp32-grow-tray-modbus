#include "DrainWaterHopper.h"
#include "Config.h"
#include "LoadCells.h"
#include "Utils.h"

// Build hopper with built-in valves on I2C address 0x0E (drain driver).
DrainWaterHopper::DrainWaterHopper(uint8_t id, Cell *loadCell)
		: id_(id),
			valve1_(Valve::Channel::B, 0x0E, Valve::Type::NormallyClosed), // EV2
			valve2_(Valve::Channel::A, 0x0E, Valve::Type::NormallyOpen),	 // EV3
			loadCell_(loadCell)
{
	for (auto &reading : last_readings_)
	{
		reading = -1000;
	}
}

void DrainWaterHopper::begin()
{
	valve1_.begin();
	valve2_.begin();
}

void DrainWaterHopper::newReading()
{
	const long reading = loadCell_->Read();

	for (size_t i = 1; i < READINGS_NUMBER; ++i)
	{
		last_readings_[i - 1] = last_readings_[i];
	}
	last_readings_[READINGS_NUMBER - 1] = reading;
}

void DrainWaterHopper::loop()
{
	newReading();
	valve1_.loop();
	valve2_.loop();

	if (!finishingDrain_)
	{
		return;
	}

	switch (finishState_)
	{
	case FinishDrainState::ClosingV3:
		valve2_.close(); // EV3
		setFinishState(FinishDrainState::OpeningV2, "OpeningV2");
		break;
	case FinishDrainState::OpeningV2:
		valve1_.open(); // EV2
		setFinishState(FinishDrainState::WaitingEmpty, "WaitingEmpty");
		break;
	case FinishDrainState::WaitingEmpty:
		if (isStableAround(0, Config_Get().tolerance))
		{
			setFinishState(FinishDrainState::ClosingV2, "ClosingV2");
		}
		break;
	case FinishDrainState::ClosingV2:
		valve1_.close(); // EV2
		if (LoadCells::instance().requestTare(2, "finish drain"))
		{
			setFinishState(FinishDrainState::WaitingTare, "WaitingTare");
		}
		break;
	case FinishDrainState::WaitingTare:
		if (!LoadCells::instance().isTareInProgress(2))
		{
			setFinishState(FinishDrainState::OpeningV3, "OpeningV3");
		}
		break;
	case FinishDrainState::OpeningV3:
		valve2_.open(); // EV3
		finishingDrain_ = false;
		setFinishState(FinishDrainState::Idle, "Idle");
		break;
	case FinishDrainState::Idle:
	default:
		finishingDrain_ = false;
		setFinishState(FinishDrainState::Idle);
		break;
	}
}

bool DrainWaterHopper::readingsSimilar(int tolerance) const
{
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

	return (max_val - min_val) <= (tolerance * 2);
}

bool DrainWaterHopper::readingsAboveMin(long min_value) const
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

bool DrainWaterHopper::readingsAboveMax(long max_value) const
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

bool DrainWaterHopper::readingsFilled() const
{
	return readingsAboveMin(-200);
}

void DrainWaterHopper::printReadings() const
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

bool DrainWaterHopper::isStableAround(long target, long tolerance) const
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

void DrainWaterHopper::setFinishState(FinishDrainState state, const char *label)
{
	finishState_ = state;
	if (label != nullptr)
	{
		Serial.println(String("Estado Drenaje cambiado a: ") + label);
	}
}

void DrainWaterHopper::finishDrain()
{
	if (finishingDrain_)
	{
		return;
	}

	finishingDrain_ = true;
	setFinishState(FinishDrainState::ClosingV3, "ClosingV3");
}

void DrainWaterHopper::openValve2()
{
	Serial.println("Abriendo valve 2");
	valve1_.open();
}

void DrainWaterHopper::closeValve2()
{
	valve1_.close();
}

void DrainWaterHopper::openValve3()
{
	valve2_.open();
}

void DrainWaterHopper::closeValve3()
{
	valve2_.close();
}

bool DrainWaterHopper::valve2IsOpen() const
{
	return valve1_.isOpen();
}

bool DrainWaterHopper::valve3IsOpen() const
{
	return valve2_.isOpen();
}

void DrainWaterHopper::resetFinishState()
{
	finishingDrain_ = false;
	setFinishState(FinishDrainState::Idle);
	Serial.println("Estado Drenaje cambiado a: Idle (reset)");
}

uint8_t DrainWaterHopper::drainFinishState() const
{
	return static_cast<uint8_t>(finishState_);
}
