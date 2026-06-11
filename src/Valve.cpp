#include "Valve.h"

std::array<Valve::DriverState, Valve::DRIVER_ADDRESS_COUNT> Valve::drivers_{};
bool Valve::wireBegun_ = false;

Valve::Valve(Channel channel, uint8_t address, Type type)
		: channel_(channel),
			address_(address),
			type_(type)
{
}

bool Valve::begin()
{
	if (!wireBegun_)
	{
		Wire.begin();
		delay(10);
		wireBegun_ = true;
	}

	DriverState &driver = driverState();
	if (!driver.begun)
	{
		driver.motor.begin(address_);
		driver.begun = true;
	}

	stop();
	begun_ = true;
	return true;
}

void Valve::open()
{
	isOpen_ = true;
	setEnergized(type_ == Type::NormallyClosed);
}

void Valve::close()
{
	isOpen_ = false;
	setEnergized(type_ == Type::NormallyOpen);
}

void Valve::loop()
{
	if (currentSpeed_ == 0)
	{
		return;
	}

	if ((millis() - energizedSinceMs_) < ENERGIZED_TIMEOUT_MS)
	{
		return;
	}

	stop();
	energizedSinceMs_ = 0;
	Serial.print("Safety stop: valve driver de-energized after ");
	Serial.print(ENERGIZED_TIMEOUT_MS);
	Serial.print(" ms at I2C 0x");
	Serial.print(address_, HEX);
	Serial.print(" channel ");
	Serial.println(channel_ == Channel::A ? "A" : "B");
}

bool Valve::isOpen() const
{
	return isOpen_;
}

int Valve::energizedSpeed() const
{
	return HOLD_SPEED;
}

uint8_t Valve::motorChannel() const
{
	return (channel_ == Channel::A) ? MOTOR1 : MOTOR2;
}

Valve::DriverState &Valve::driverState()
{
	const size_t index = (address_ < DRIVER_ADDRESS_COUNT) ? address_ : (DRIVER_ADDRESS_COUNT - 1);
	return drivers_[index];
}

void Valve::setEnergized(bool energized)
{
	if (!begun_)
	{
		begin();
	}

	if (!energized)
	{
		stop();
		energizedSinceMs_ = 0;
		return;
	}

	const int speed = energizedSpeed();
	if (currentSpeed_ == speed)
	{
		return;
	}

	currentSpeed_ = speed;
	energizedSinceMs_ = millis();
	driverState().motor.speed(motorChannel(), speed);
}

void Valve::setOpenStateFromEnergized(bool energized)
{
	isOpen_ = (type_ == Type::NormallyClosed) ? energized : !energized;
}

void Valve::stop()
{
	driverState().motor.stop(motorChannel());
	currentSpeed_ = 0;
	setOpenStateFromEnergized(false);
}
