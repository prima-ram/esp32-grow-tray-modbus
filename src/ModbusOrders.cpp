#include "ModbusOrders.h"

#include "Config.h"
#include "DrainWaterHopper.h"
#include "LoadCells.h"
#include "WaterInletHopper.h"

namespace
{
// ESP32 configuration
constexpr uint8_t MODBUS_SLAVE_ID = 2;
constexpr uint8_t MODBUS_RX_PIN = 8;
constexpr uint8_t MODBUS_TX_PIN = 9;
constexpr uint8_t PRESSURE_ADC_PIN = A8;
constexpr uint32_t PRESSURE_UPDATE_INTERVAL_MS = 1000;

// Modbus readings
constexpr uint16_t REG_WEIGHT_IRRIGATION_G = 1;
constexpr uint16_t REG_WEIGHT_DRAIN_G = 2;
constexpr uint16_t REG_PRESSURE_CBAR = 3;

// Modbus actions
constexpr uint16_t REG_TARE_CELL_1 = 100;
constexpr uint16_t REG_TARE_CELL_2 = 101;
constexpr uint16_t REG_CAL_WEIGHT_CELL_1_DG = 102;
constexpr uint16_t REG_CAL_TRIGGER_CELL_1 = 103;
constexpr uint16_t REG_CAL_WEIGHT_CELL_2_DG = 104;
constexpr uint16_t REG_CAL_TRIGGER_CELL_2 = 105;
constexpr uint16_t REG_ACTION_FINISH_IRRIGATION = 106;
constexpr uint16_t REG_ACTION_FINISH_DRAIN = 107;
constexpr uint16_t REG_ACTION_RESET_DEVICE = 108;

// Modbus secondary actions and status for debugging
constexpr uint16_t REG_CMD_VALVE_1 = 200;
constexpr uint16_t REG_CMD_VALVE_2 = 201;
constexpr uint16_t REG_CMD_VALVE_3 = 202;
constexpr uint16_t REG_PRESSURE_MV = 203;
constexpr uint16_t REG_PRESSURE_MA_X100 = 204;
constexpr uint16_t REG_PRESSURE_ADC_RAW = 205;
constexpr uint16_t REG_LOAD_CELL_1_ERROR = 206;
constexpr uint16_t REG_LOAD_CELL_2_ERROR = 207;
constexpr uint16_t REG_VALVE_COMMAND_STATUS = 208;

// Trigger for tare and calibration actions: when the corresponding register is written with these values, the action is triggered and the register is reset to 0 by ModbusOrders.
constexpr uint16_t TARE_TRIGGER_VALUE = 5;
constexpr uint16_t CALIBRATION_TRIGGER_VALUE = 1;
constexpr uint16_t RESET_TRIGGER_VALUE = 1;

// Modbus register map: registers 1-208 are available for use, starting from REG_WEIGHT_IRRIGATION_G. Registers above 208 are not defined and will be ignored by the Modbus library.
constexpr uint16_t MODBUS_FIRST_HREG = 1;
constexpr uint16_t MODBUS_HREG_COUNT = 208;
} // namespace

ModbusOrders::ModbusOrders(LoadCells &loadCells, WaterInletHopper &waterInlet, DrainWaterHopper &drain)
		: loadCells_(loadCells), waterInlet_(waterInlet), drain_(drain)
{
}

void ModbusOrders::begin()
{
	Serial1.begin(9600, SERIAL_8N1, MODBUS_RX_PIN, MODBUS_TX_PIN);
	mb_.begin(&Serial1);
	mb_.slave(MODBUS_SLAVE_ID);
	mb_.addHreg(MODBUS_FIRST_HREG, 0, MODBUS_HREG_COUNT);
	mb_.Hreg(REG_CMD_VALVE_1, lastValveCommands_[0]);
	mb_.Hreg(REG_CMD_VALVE_2, lastValveCommands_[1]);
	mb_.Hreg(REG_CMD_VALVE_3, lastValveCommands_[2]);
	mb_.Hreg(REG_VALVE_COMMAND_STATUS, 0);
}

void ModbusOrders::task()
{
	mb_.task();
}

void ModbusOrders::process(uint32_t nowMs)
{
	updatePressure(nowMs);
	processActionRegisters();
	processValveCommandRegisters();
	syncValveCommandRegisters();
}

void ModbusOrders::updateWeights(float irrigationGrams, float drainGrams)
{
	writeSignedHreg(REG_WEIGHT_IRRIGATION_G, isnan(irrigationGrams) ? 0 : clampToInt16(lroundf(irrigationGrams)));
	writeSignedHreg(REG_WEIGHT_DRAIN_G, isnan(drainGrams) ? 0 : clampToInt16(lroundf(drainGrams)));
	mb_.Hreg(REG_LOAD_CELL_1_ERROR, isnan(irrigationGrams) ? 1 : 0);
	mb_.Hreg(REG_LOAD_CELL_2_ERROR, isnan(drainGrams) ? 1 : 0);
}

void ModbusOrders::updatePressure(uint32_t nowMs, bool force)
{
	if (!force && nowMs - lastPressureUpdateMs_ < PRESSURE_UPDATE_INTERVAL_MS)
	{
		return;
	}

	lastPressureUpdateMs_ = nowMs;
	const PressureReading pressure = readPressure();
	writeSignedHreg(REG_PRESSURE_CBAR, pressure.centibar);
	mb_.Hreg(REG_PRESSURE_MV, pressure.millivolts);
	mb_.Hreg(REG_PRESSURE_MA_X100, pressure.currentMaX100);
	mb_.Hreg(REG_PRESSURE_ADC_RAW, pressure.adc);
}

void ModbusOrders::syncValveCommandRegisters()
{
	lastValveCommands_[0] = boolRegister(waterInlet_.valve1IsOpen());
	lastValveCommands_[1] = boolRegister(drain_.valve2IsOpen());
	lastValveCommands_[2] = boolRegister(drain_.valve3IsOpen());
	mb_.Hreg(REG_CMD_VALVE_1, lastValveCommands_[0]);
	mb_.Hreg(REG_CMD_VALVE_2, lastValveCommands_[1]);
	mb_.Hreg(REG_CMD_VALVE_3, lastValveCommands_[2]);
}

void ModbusOrders::setValveCommandRegister(uint8_t valveIndex, uint16_t value)
{
	if (valveIndex >= 3)
	{
		return;
	}

	lastValveCommands_[valveIndex] = value;
	mb_.Hreg(REG_CMD_VALVE_1 + valveIndex, value);
}

int16_t ModbusOrders::clampToInt16(long value)
{
	if (value > INT16_MAX)
	{
		return INT16_MAX;
	}
	if (value < INT16_MIN)
	{
		return INT16_MIN;
	}
	return static_cast<int16_t>(value);
}

uint16_t ModbusOrders::boolRegister(bool value)
{
	return value ? 1 : 0;
}

void ModbusOrders::writeSignedHreg(uint16_t reg, int16_t value)
{
	mb_.Hreg(reg, static_cast<uint16_t>(value));
}

ModbusOrders::PressureReading ModbusOrders::readPressure() const
{
	const int adc = analogRead(PRESSURE_ADC_PIN);
	const float voltage = static_cast<float>(adc) * 3.62f / 4095.0f;
	const float currentMa = voltage / 100.0f * 1000.0f;
	const float pressureBar = (currentMa - 4.0f) * 10.0f / 16.0f;

	PressureReading reading;
	reading.adc = static_cast<uint16_t>(constrain(adc, 0, 4095));
	reading.millivolts = static_cast<uint16_t>(constrain(lroundf(voltage * 1000.0f), 0L, 65535L));
	reading.currentMaX100 = static_cast<uint16_t>(constrain(lroundf(currentMa * 100.0f), 0L, 65535L));
	reading.centibar = clampToInt16(lroundf(pressureBar * 100.0f));
	return reading;
}

void ModbusOrders::processActionRegisters()
{
	if (mb_.Hreg(REG_TARE_CELL_1) == TARE_TRIGGER_VALUE)
	{
		loadCells_.requestTare(1, "Modbus");
		mb_.Hreg(REG_TARE_CELL_1, 0);
	}

	if (mb_.Hreg(REG_TARE_CELL_2) == TARE_TRIGGER_VALUE)
	{
		loadCells_.requestTare(2, "Modbus");
		mb_.Hreg(REG_TARE_CELL_2, 0);
	}

	if (mb_.Hreg(REG_CAL_TRIGGER_CELL_1) == CALIBRATION_TRIGGER_VALUE)
	{
		const float grams = static_cast<float>(mb_.Hreg(REG_CAL_WEIGHT_CELL_1_DG)) / 10.0f;
		if (grams > 0.0f)
		{
			Config &cfg = Config_Get();
			cfg.factor1 = loadCells_.calibrate(1, grams);
			Config_Save();
			Serial.printf("Modbus calibrated cell 1 with %.1f g\n", static_cast<double>(grams));
		}
		mb_.Hreg(REG_CAL_TRIGGER_CELL_1, 0);
	}

	if (mb_.Hreg(REG_CAL_TRIGGER_CELL_2) == CALIBRATION_TRIGGER_VALUE)
	{
		const float grams = static_cast<float>(mb_.Hreg(REG_CAL_WEIGHT_CELL_2_DG)) / 10.0f;
		if (grams > 0.0f)
		{
			Config &cfg = Config_Get();
			cfg.factor2 = loadCells_.calibrate(2, grams);
			Config_Save();
			Serial.printf("Modbus calibrated cell 2 with %.1f g\n", static_cast<double>(grams));
		}
		mb_.Hreg(REG_CAL_TRIGGER_CELL_2, 0);
	}

	if (mb_.Hreg(REG_ACTION_RESET_DEVICE) == RESET_TRIGGER_VALUE)
	{
		Serial.println("Modbus triggered device reset");
		mb_.Hreg(REG_ACTION_RESET_DEVICE, 0);
		ESP.restart();
	}
}

void ModbusOrders::processValveCommandRegisters()
{
	for (uint8_t valveIndex = 0; valveIndex < 3; ++valveIndex)
	{
		const uint16_t command = mb_.Hreg(REG_CMD_VALVE_1 + valveIndex);
		if (command != lastValveCommands_[valveIndex])
		{
			applyValveCommand(valveIndex, command);
		}
	}
}

void ModbusOrders::applyValveCommand(uint8_t valveIndex, uint16_t command)
{
	if (command > 1)
	{
		mb_.Hreg(REG_VALVE_COMMAND_STATUS, valveIndex + 1);
		mb_.Hreg(REG_CMD_VALVE_1 + valveIndex, lastValveCommands_[valveIndex]);
		Serial.printf("Invalid Modbus valve command V%u=%u\n", valveIndex + 1, command);
		return;
	}

	if (command == 1)
	{
		if (valveIndex == 0)
		{
			waterInlet_.openValve1();
		}
		else if (valveIndex == 1)
		{
			drain_.openValve2();
		}
		else
		{
			drain_.openValve3();
		}
	}
	else
	{
		if (valveIndex == 0)
		{
			waterInlet_.closeValve1();
		}
		else if (valveIndex == 1)
		{
			drain_.closeValve2();
		}
		else
		{
			drain_.closeValve3();
		}
	}

	lastValveCommands_[valveIndex] = command;
	mb_.Hreg(REG_VALVE_COMMAND_STATUS, 0);
	syncValveCommandRegisters();
}
