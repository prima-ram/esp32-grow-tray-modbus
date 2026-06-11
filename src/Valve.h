#pragma once

#include <array>
#include <Arduino.h>
#include <Wire.h>
#include <Grove_I2C_Motor_Driver.h>

class Valve
{
public:
	enum class Channel : uint8_t
	{
		A = 0,
		B = 1
	};

	enum class Type : uint8_t
	{
		NormallyClosed = 0,
		NormallyOpen = 1
	};

	Valve(Channel channel, uint8_t address = 0x0F, Type type = Type::NormallyClosed);

	bool begin();
	void open();
	void close();
	void loop();
	bool isOpen() const;

private:
	struct DriverState
	{
		I2CMotorDriver motor;
		bool begun = false;
	};

	static constexpr size_t DRIVER_ADDRESS_COUNT = 0x10;
	static constexpr int HOLD_SPEED = -255;
	static constexpr unsigned long ENERGIZED_TIMEOUT_MS = 10UL * 60UL * 1000UL;
	static std::array<DriverState, DRIVER_ADDRESS_COUNT> drivers_;
	static bool wireBegun_;

	int energizedSpeed() const;
	uint8_t motorChannel() const;
	DriverState &driverState();
	void setEnergized(bool energized);
	void setOpenStateFromEnergized(bool energized);
	void stop();

	bool begun_ = false;
	bool isOpen_ = false;
	int currentSpeed_ = 0;
	unsigned long energizedSinceMs_ = 0;

	Channel channel_;
	uint8_t address_;
	Type type_;
};
