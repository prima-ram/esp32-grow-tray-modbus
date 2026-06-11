#include "LoadCells.h"
#include "Config.h"
#include "WaterInletHopper.h"
#include "DrainWaterHopper.h"
#include <Wire.h>

namespace
{
void scanI2C()
{
	Serial.println("I2C scan start");
	Wire.begin();
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

void pingI2C(uint8_t addr)
{
	Wire.begin();
	Wire.beginTransmission(addr);
	const uint8_t err = Wire.endTransmission();
	Serial.printf("I2C ping 0x%02X -> %u\n", addr, err);
}
} // namespace

LoadCells &LoadCells::instance()
{
	static LoadCells inst;
	return inst;
}

LoadCells::LoadCells()
		: loadCell1(PIN_DOUT1, PIN_SCK1),
			loadCell2(PIN_DOUT2, PIN_SCK2)
{
	for (uint8_t sensor = 0; sensor < 2; ++sensor)
	{
		recentCounts_[sensor] = 0;
		recentWriteIndex_[sensor] = 0;
		for (uint8_t i = 0; i < LORAWAN_HISTORY_SIZE; ++i)
		{
			recentReadings_[sensor][i] = 0;
		}
	}
}

void LoadCells::begin()
{
	Config &cfg = Config_Get();
	cfgPtr = &cfg;
	loadCell1.SetOffset(cfg.offset1);
	loadCell1.SetFactor(cfg.factor1);
	loadCell2.SetOffset(cfg.offset2);
	loadCell2.SetFactor(cfg.factor2);
}

bool LoadCells::update()
{
	const bool updated1 = loadCell1.Update() != 0;
	const bool updated2 = loadCell2.Update() != 0;

	if (updated1)
	{
		pushReading(1, loadCell1.Read());
	}
	if (updated2)
	{
		pushReading(2, loadCell2.Read());
	}

	if (pendingTares_[0].active && loadCell1.TareComplete())
	{
		completePendingTare(1);
	}

	if (pendingTares_[1].active && loadCell2.TareComplete())
	{
		completePendingTare(2);
	}

	return updated1 || updated2;
}

Cell &LoadCells::cellFromId(uint8_t sensorId)
{
	if (sensorId == 1)
		return loadCell1;
	else
		return loadCell2;
}

long LoadCells::read(uint8_t sensorId)
{
	return cellFromId(sensorId).Read();
}

void LoadCells::pushReading(uint8_t sensorId, long reading)
{
	const uint8_t sensorIdx = (sensorId == 2) ? 1 : 0;
	recentReadings_[sensorIdx][recentWriteIndex_[sensorIdx]] = reading;
	recentWriteIndex_[sensorIdx] = (recentWriteIndex_[sensorIdx] + 1) % LORAWAN_HISTORY_SIZE;
	if (recentCounts_[sensorIdx] < LORAWAN_HISTORY_SIZE)
	{
		++recentCounts_[sensorIdx];
	}
}

uint8_t LoadCells::recentReadingsCount(uint8_t sensorId) const
{
	const uint8_t sensorIdx = (sensorId == 2) ? 1 : 0;
	return recentCounts_[sensorIdx];
}

long LoadCells::recentReading(uint8_t sensorId, uint8_t index) const
{
	const uint8_t sensorIdx = (sensorId == 2) ? 1 : 0;
	const uint8_t count = recentCounts_[sensorIdx];
	if (index >= count)
	{
		return 0;
	}

	const uint8_t writeIdx = recentWriteIndex_[sensorIdx];
	const uint8_t oldestIdx = (writeIdx + LORAWAN_HISTORY_SIZE - count) % LORAWAN_HISTORY_SIZE;
	const uint8_t pos = (oldestIdx + index) % LORAWAN_HISTORY_SIZE;
	return recentReadings_[sensorIdx][pos];
}

void LoadCells::setFactor(uint8_t sensorId, float factor)
{
	cellFromId(sensorId).SetFactor(factor);
}

void LoadCells::setOffset(uint8_t sensorId, long offset)
{
	cellFromId(sensorId).SetOffset(offset);
}

bool LoadCells::requestTare(uint8_t sensorId, const char *source)
{
	if (sensorId < 1 || sensorId > 2)
	{
		Serial.printf("Cannot start tare for invalid sensor %u\n", sensorId);
		return false;
	}

	const uint8_t sensorIdx = sensorId - 1;
	if (pendingTares_[sensorIdx].active)
	{
		Serial.printf("Tare already in progress for sensor %u\n", sensorId);
		return false;
	}

	pendingTares_[sensorIdx].active = true;
	pendingTares_[sensorIdx].source = (source != nullptr) ? source : "unknown";
	cellFromId(sensorId).StartTare();
	Serial.printf("Tare started from %s for sensor %u\n",
								pendingTares_[sensorIdx].source,
								sensorId);
	return true;
}

bool LoadCells::isTareInProgress(uint8_t sensorId) const
{
	if (sensorId < 1 || sensorId > 2)
	{
		return false;
	}

	return pendingTares_[sensorId - 1].active;
}

float LoadCells::calibrate(uint8_t sensorId, float grams)
{
	return cellFromId(sensorId).Calibrate(grams);
}

void LoadCells::refreshIfDue(uint32_t intervalMs)
{
	const unsigned long now = millis();

	if (!refreshInProgress_ && (now - lastRefreshMs >= intervalMs))
	{
		loadCell1.StartRefreshDataSet();
		loadCell2.StartRefreshDataSet();
		refreshInProgress_ = true;
		refreshCell1Pending_ = true;
		refreshCell2Pending_ = true;
		lastRefreshMs = now;
	}

	if (!refreshInProgress_)
	{
		return;
	}

	if (refreshCell1Pending_ && loadCell1.RefreshDataSetComplete())
	{
		refreshCell1Pending_ = false;
	}

	if (refreshCell2Pending_ && loadCell2.RefreshDataSetComplete())
	{
		refreshCell2Pending_ = false;
	}

	if (!refreshCell1Pending_ && !refreshCell2Pending_)
	{
		refreshInProgress_ = false;
	}
}

void LoadCells::completePendingTare(uint8_t sensorId)
{
	if (cfgPtr == nullptr || sensorId < 1 || sensorId > 2)
	{
		return;
	}

	const uint8_t sensorIdx = sensorId - 1;
	PendingTare &pending = pendingTares_[sensorIdx];
	Cell &cell = cellFromId(sensorId);
	const long offset = cell.TareOffset();

	if (sensorId == 1)
	{
		cfgPtr->offset1 = offset;
	}
	else
	{
		cfgPtr->offset2 = offset;
	}

	cell.SetOffset(offset);
	Config_Save();
	Serial.printf("Tare completed from %s for sensor %u. Offset: %ld\n",
								pending.source != nullptr ? pending.source : "unknown",
								sensorId,
								offset);
	pending.active = false;
	pending.source = "unknown";
}

void LoadCells::processCommand(const String &data, WaterInletHopper *waterInlet, DrainWaterHopper *drain)
{
	Serial.println("Configuring");
	if (data.length() == 0 || cfgPtr == nullptr)
		return;

	Config &cfg = *cfgPtr;
	String upper = data;
	upper.trim();
	upper.toUpperCase();
 
	Serial.print("[CMD] ");
	Serial.print("Recibido: ");
	Serial.println(data);

	if (upper == "I2C_SCAN")
	{
		scanI2C();
		return;
	}
	if (upper.startsWith("I2C_PING_"))
	{
		const String addrStr = upper.substring(strlen("I2C_PING_"));
		char *end = nullptr;
		const uint8_t addr = static_cast<uint8_t>(strtoul(addrStr.c_str(), &end, 0));
		if (end != addrStr.c_str())
		{
			pingI2C(addr);
		}
		else
		{
			Serial.println("I2C_PING_: direccion invalida");
		}
		return;
	}

	if (upper == "TEST_PULSE_ALL")
	{
		debugWaterInlet_ = waterInlet;
		debugDrain_ = drain;
		if (debugWaterInlet_ == nullptr || debugDrain_ == nullptr)
		{
			Serial.println("TEST_PULSE_ALL: hopper no disponible");
			return;
		}
		debugPulseState_ = DebugPulseState::V1Open;
		debugPulseNextMs_ = millis();
		Serial.println("TEST_PULSE_ALL: sequence started");
		return;
	}

	if (upper == "TEST_PULSE_V1" || upper == "TEST_PULSE_V1_OPEN" || upper == "TEST_PULSE_V1_CLOSE")
	{
		if (waterInlet == nullptr)
		{
			Serial.println("TEST_PULSE_V1: hopper no disponible");
			return;
		}
		if (upper.endsWith("_CLOSE"))
		{
			waterInlet->closeValve1();
			Serial.println("TEST_PULSE_V1_CLOSE sent");
		}
		else
		{
			waterInlet->openValve1();
			Serial.println("TEST_PULSE_V1_OPEN sent");
		}
		return;
	}

	if (upper == "TEST_PULSE_V2" || upper == "TEST_PULSE_V2_OPEN" || upper == "TEST_PULSE_V2_CLOSE")
	{
		if (drain == nullptr)
		{
			Serial.println("TEST_PULSE_V2: hopper no disponible");
			return;
		}
		if (upper.endsWith("_CLOSE"))
		{
			drain->closeValve2();
			Serial.println("TEST_PULSE_V2_CLOSE sent");
		}
		else
		{
			drain->openValve2();
			Serial.println("TEST_PULSE_V2_OPEN sent");
		}
		return;
	}

	if (upper == "TEST_PULSE_V3" || upper == "TEST_PULSE_V3_OPEN" || upper == "TEST_PULSE_V3_CLOSE")
	{
		if (drain == nullptr)
		{
			Serial.println("TEST_PULSE_V3: hopper no disponible");
			return;
		}
		if (upper.endsWith("_CLOSE"))
		{
			drain->closeValve3();
			Serial.println("TEST_PULSE_V3_CLOSE sent");
		}
		else
		{
			drain->openValve3();
			Serial.println("TEST_PULSE_V3_OPEN sent");
		}
		return;
	}

	if (upper == "OPEN_V1" || upper == "CLOSE_V1" ||
			upper == "OPEN_V2" || upper == "CLOSE_V2" ||
			upper == "OPEN_V3" || upper == "CLOSE_V3")
	{
		if ((upper == "OPEN_V1" || upper == "CLOSE_V1") && waterInlet != nullptr)
		{
			if (upper == "OPEN_V1")
			{
				waterInlet->openValve1();
				Serial.println("Valve 1 opened");
			}
			else
			{
				waterInlet->closeValve1();
				Serial.println("Valve 1 closed");
			}
		}
		else if ((upper == "OPEN_V2" || upper == "CLOSE_V2") && drain != nullptr)
		{
			if (upper == "OPEN_V2")
			{
				drain->openValve2();
				Serial.println("Valve 2 opened");
			}
			else
			{
				drain->closeValve2();
				Serial.println("Valve 2 closed");
			}
		}
		else if ((upper == "OPEN_V3" || upper == "CLOSE_V3") && drain != nullptr)
		{
			if (upper == "OPEN_V3")
			{
				drain->openValve3();
				Serial.println("Valve 3 opened");
			}
			else
			{
				drain->closeValve3();
				Serial.println("Valve 3 closed");
			}
		}
		else
		{
			Serial.println("Error: hopper no disponible para comando de valvula");
		}
		return;
	}

	if (upper == "TARE_1")
	{
		if (requestTare(1, "serial"))
		{
			Serial.println("Tara del peso 1 iniciada");
		}
	}
	else if (upper == "TARE_2")
	{
		if (requestTare(2, "serial"))
		{
			Serial.println("Tara del peso 2 iniciada");
		}
	}
	else if (upper == "TARE_BOTH")
	{
		bool started = false;
		started = requestTare(1, "serial") || started;
		started = requestTare(2, "serial") || started;
		if (started)
		{
			Serial.println("Taras de los pesos 1 y 2 iniciadas");
		}
	}
	else if (upper.startsWith("CALIBRATE"))
	{
		uint8_t id = 0;
		String numberStr = data;

		if (upper.startsWith("CALIBRATE_1"))
		{
			id = 1;
			const int separatorIndex = data.indexOf('-');
			numberStr = data.substring(separatorIndex + 1);
		}
		else if (upper.startsWith("CALIBRATE_2"))
		{
			id = 2;
			const int separatorIndex = data.indexOf('-');
			numberStr = data.substring(separatorIndex + 1);
		}

		if (numberStr.length() == 0 || !isNumber(numberStr))
		{
			Serial.println("Error: debe ser un numero");
			return;
		}

		const float grams = numberStr.toFloat();

		if (id == 1 || id == 2)
		{
			const float cal = calibrate(id, grams);
			if (id == 1)
			{
				cfg.factor1 = cal;
				Serial.print("Factor 1: ");
			}
			else
			{
				cfg.factor2 = cal;
				Serial.print("Factor 2: ");
			}
			Serial.println(cal);
			Config_Save();
			Serial.println("Calibracion completada");
		}
		else
		{
			Serial.println("Error: sensorId no valido para calibrar");
		}
	}
	else
	{
		Serial.println("Comando desconocido");
	}
}

void LoadCells::debugLoop()
{
	if (debugPulseState_ == DebugPulseState::Idle)
	{
		return;
	}
	const uint32_t now = millis();
	if (now < debugPulseNextMs_)
	{
		return;
	}

	switch (debugPulseState_)
	{
	case DebugPulseState::V1Open:
		if (debugWaterInlet_ != nullptr)
		{
			debugWaterInlet_->openValve1();
			Serial.println("TEST_PULSE_ALL: V1 open");
		}
		debugPulseState_ = DebugPulseState::V1Close;
		break;
	case DebugPulseState::V1Close:
		if (debugWaterInlet_ != nullptr)
		{
			debugWaterInlet_->closeValve1();
			Serial.println("TEST_PULSE_ALL: V1 close");
		}
		debugPulseState_ = DebugPulseState::V2Open;
		break;
	case DebugPulseState::V2Open:
		if (debugDrain_ != nullptr)
		{
			debugDrain_->openValve2();
			Serial.println("TEST_PULSE_ALL: V2 open");
		}
		debugPulseState_ = DebugPulseState::V2Close;
		break;
	case DebugPulseState::V2Close:
		if (debugDrain_ != nullptr)
		{
			debugDrain_->closeValve2();
			Serial.println("TEST_PULSE_ALL: V2 close");
		}
		debugPulseState_ = DebugPulseState::V3Open;
		break;
	case DebugPulseState::V3Open:
		if (debugDrain_ != nullptr)
		{
			debugDrain_->openValve3();
			Serial.println("TEST_PULSE_ALL: V3 open");
		}
		debugPulseState_ = DebugPulseState::V3Close;
		break;
	case DebugPulseState::V3Close:
		if (debugDrain_ != nullptr)
		{
			debugDrain_->closeValve3();
			Serial.println("TEST_PULSE_ALL: V3 close");
		}
		debugPulseState_ = DebugPulseState::Idle;
		Serial.println("TEST_PULSE_ALL: sequence done");
		break;
	default:
		debugPulseState_ = DebugPulseState::Idle;
		break;
	}

	debugPulseNextMs_ = now + debugPulseStepMs_;
}

bool LoadCells::isNumber(const String &str)
{
	bool dotSeen = false;
	bool digitSeen = false;

	for (unsigned int i = 0; i < str.length(); i++)
	{
		const char c = str[i];
		if (i == 0 && c == '-')
		{
			continue;
		}
		if (c == '.')
		{
			if (dotSeen)
			{
				return false;
			}
			dotSeen = true;
			continue;
		}
		if (!isDigit(c))
		{
			return false;
		}
		digitSeen = true;
	}
	return digitSeen;
}
