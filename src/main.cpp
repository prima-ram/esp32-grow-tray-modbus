#include <Arduino.h>
#include <Wire.h>

#include "Config.h"
#include "DrainWaterHopper.h"
#include "LoadCells.h"
#include "ModbusOrders.h"
#include "WaterInletHopper.h"

namespace
{
constexpr uint32_t WEIGHT_LOG_INTERVAL_MS = 3000;
constexpr uint32_t CONFIG_LOG_INTERVAL_MS = 30000;
constexpr size_t SERIAL_COMMAND_BUFFER_SIZE = 96;

LoadCells &loadCells = LoadCells::instance();
WaterInletHopper hopper1(1, &loadCells.cellFromId(1));
DrainWaterHopper hopper2(2, &loadCells.cellFromId(2));
ModbusOrders modbusOrders(loadCells, hopper1, hopper2);

uint32_t lastWeightLogMs = 0;
uint32_t lastConfigLogMs = 0;
float lastWeight1 = NAN;
float lastWeight2 = NAN;
bool hasWeight = false;
char serialCommandBuffer[SERIAL_COMMAND_BUFFER_SIZE] = {};
size_t serialCommandLength = 0;

void printConfig()
{
	const Config &cfg = Config_Get();
	Serial.printf("Config mac=%s magic=0x%08lX offset1=%ld factor1=%.6f offset2=%ld factor2=%.6f tolerance=%u\n",
								cfg.mac.c_str(),
								static_cast<unsigned long>(cfg.magic),
								cfg.offset1,
								static_cast<double>(cfg.factor1),
								cfg.offset2,
								static_cast<double>(cfg.factor2),
								static_cast<unsigned>(cfg.tolerance));
}

void scanI2C()
{
	Serial.println("I2C scan start");
	uint8_t count = 0;
	for (uint8_t addr = 1; addr < 127; ++addr)
	{
		Wire.beginTransmission(addr);
		if (Wire.endTransmission() == 0)
		{
			Serial.printf("I2C device found at 0x%02X\n", addr);
			++count;
		}
	}
	if (count == 0)
	{
		Serial.println("I2C scan: no devices found");
	}
	else
	{
		Serial.printf("I2C scan done: %u device(s)\n", count);
	}
}

void executeSerialCommand(const String &data)
{
	String command = data;
	command.trim();
	if (command.length() == 0)
	{
		return;
	}

	String upper = data;
	upper.trim();
	upper.toUpperCase();

	if (upper == "I2C_SCAN")
	{
		scanI2C();
	}
	else if (upper == "OPEN_V1")
	{
		hopper1.openValve1();
		modbusOrders.setValveCommandRegister(0, 1);
		Serial.println("Valve 1 open");
	}
	else if (upper == "CLOSE_V1")
	{
		hopper1.closeValve1();
		modbusOrders.setValveCommandRegister(0, 0);
		Serial.println("Valve 1 close");
	}
	else if (upper == "OPEN_V2")
	{
		hopper2.openValve2();
		modbusOrders.setValveCommandRegister(1, 1);
		Serial.println("Valve 2 open");
	}
	else if (upper == "CLOSE_V2")
	{
		hopper2.closeValve2();
		modbusOrders.setValveCommandRegister(1, 0);
		Serial.println("Valve 2 close");
	}
	else if (upper == "OPEN_V3")
	{
		hopper2.openValve3();
		modbusOrders.setValveCommandRegister(2, 1);
		Serial.println("Valve 3 open");
	}
	else if (upper == "CLOSE_V3")
	{
		hopper2.closeValve3();
		modbusOrders.setValveCommandRegister(2, 0);
		Serial.println("Valve 3 close");
	}
	else
	{
		loadCells.processCommand(command, &hopper1, &hopper2);
	}
	modbusOrders.syncValveCommandRegisters();
}

void handleSerialCommand()
{
	while (Serial.available() > 0)
	{
		const char c = static_cast<char>(Serial.read());
		if (c == '\r')
		{
			continue;
		}

		if (c == '\n')
		{
			serialCommandBuffer[serialCommandLength] = '\0';
			executeSerialCommand(String(serialCommandBuffer));
			serialCommandLength = 0;
			serialCommandBuffer[0] = '\0';
			continue;
		}

		if (serialCommandLength < SERIAL_COMMAND_BUFFER_SIZE - 1)
		{
			serialCommandBuffer[serialCommandLength++] = c;
			serialCommandBuffer[serialCommandLength] = '\0';
		}
		else
		{
			serialCommandLength = 0;
			serialCommandBuffer[0] = '\0';
			Serial.println("Serial command too long; discarded");
		}
	}
}

} // namespace

void setup()
{
	Serial.begin(115200);
	Serial.println("boot");

	Config_Load();
	loadCells.begin();
	const Config &cfg = Config_Get();
	Serial.println("Factory MAC: " + cfg.mac);
	printConfig();

	hopper1.begin();
	hopper2.begin();
	hopper1.closeValve1();
	hopper2.closeValve2();
	hopper2.openValve3();
	hopper1.finishIrrigation();
	hopper2.finishDrain();
	Serial.println("Startup drain sequence triggered");

	modbusOrders.begin();
	modbusOrders.syncValveCommandRegisters();
	modbusOrders.updateWeights(lastWeight1, lastWeight2);
	modbusOrders.updatePressure(millis(), true);
	Serial.println("Setup completed");
}

void loop()
{
	hopper1.loop();
	hopper2.loop();
	modbusOrders.syncValveCommandRegisters();

	modbusOrders.task();

	const bool newData = loadCells.update();
	if (newData)
	{
		lastWeight1 = loadCells.read(1);
		lastWeight2 = loadCells.read(2);
		hasWeight = true;
		modbusOrders.updateWeights(lastWeight1, lastWeight2);
	}

	const uint32_t nowMs = millis();
	modbusOrders.process(nowMs);

	if (hasWeight && (nowMs - lastWeightLogMs >= WEIGHT_LOG_INTERVAL_MS))
	{
		lastWeightLogMs = nowMs;
		Serial.printf("Peso 1: %.2f | Peso 2: %.2f\n", lastWeight1, lastWeight2);
	}

	if ((nowMs - lastConfigLogMs) >= CONFIG_LOG_INTERVAL_MS)
	{
		lastConfigLogMs = nowMs;
		printConfig();
	}

	handleSerialCommand();
	loadCells.refreshIfDue();
}
