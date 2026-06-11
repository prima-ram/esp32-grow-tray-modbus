#pragma once

#include <Arduino.h>
#include <ModbusRTU.h>

class DrainWaterHopper;
class LoadCells;
class WaterInletHopper;

class ModbusOrders
{
public:
	explicit ModbusOrders(LoadCells &loadCells, WaterInletHopper &waterInlet, DrainWaterHopper &drain);

	void begin();
	void task();
	void process(uint32_t nowMs);
	void updateWeights(float irrigationGrams, float drainGrams);
	void updatePressure(uint32_t nowMs, bool force = false);
	void syncValveCommandRegisters();
	void setValveCommandRegister(uint8_t valveIndex, uint16_t value);

private:
	struct PressureReading
	{
		uint16_t adc = 0;
		uint16_t millivolts = 0;
		uint16_t currentMaX100 = 0;
		int16_t centibar = 0;
	};

	static int16_t clampToInt16(long value);
	static uint16_t boolRegister(bool value);

	void writeSignedHreg(uint16_t reg, int16_t value);
	PressureReading readPressure() const;
	void processActionRegisters();
	void processValveCommandRegisters();
	void applyValveCommand(uint8_t valveIndex, uint16_t command);

	LoadCells &loadCells_;
	WaterInletHopper &waterInlet_;
	DrainWaterHopper &drain_;
	ModbusRTU mb_;
	uint32_t lastPressureUpdateMs_ = 0;
	uint16_t lastValveCommands_[3] = {0, 0, 1};
};
