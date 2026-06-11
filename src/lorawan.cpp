#include "LoadCells.h"
#include "lorawan.h"

#include <LoRaWan-Arduino.h>
#include <SPI.h>
#include <esp_system.h>
#include <Arduino.h>
#include <WiFi.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "lorawan_keys.h"
#include "radio_pins.h"
#include "Config.h"

#define LORAWAN_APP_DATA_BUFF_SIZE 64
#define LORAWAN_APP_PORT 2
#define LORAWAN_APP_TX_DUTYCYCLE 60000 // ms
#define APP_TX_DUTYCYCLE_RND 1000
#define JOINREQ_NBTRIALS 5

void lorawan_has_joined_handler(void);
void lorawan_rx_handler(lmh_app_data_t *app_data);
void lorawan_confirm_class_handler(DeviceClass_t Class);
void lorawan_join_failed_handler(void);
void tx_lora_periodic_handler(void);
uint32_t timers_init(void);
void fillDevEuiFromMac(uint8_t *devEui);
void printDevEui(const uint8_t *devEui);
int16_t readInternalTempC();
void formatDevEuiHex(char *out);
void sendGramsFrame();
void buildGramsPayload(bool increaseCounter);
bool parseLoraCommand(const String &payload, LoraCommand &out);
const char *commandName(LoraCommandType type);
void attemptJoin();
bool extractCommandPayload(String &payload);

namespace
{
	hw_config hwConfig{};
	TimerEvent_t appTimer;

	uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];
	lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0};

	lmh_param_t lora_param_init = {LORAWAN_ADR_ON, LORAWAN_DEFAULT_DATARATE, LORAWAN_PUBLIC_NETWORK, JOINREQ_NBTRIALS, LORAWAN_DEFAULT_TX_POWER, LORAWAN_DUTYCYCLE_OFF};

	lmh_callback_t lora_callbacks = {BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
																	 lorawan_rx_handler, lorawan_has_joined_handler,
																	 lorawan_confirm_class_handler, lorawan_join_failed_handler};

	LoRaMacRegion_t currentRegion = LORAMAC_REGION_EU868;
	LoraCommandHandler commandHandler = nullptr;
	long lastLoraCommandId = -1;
	uint8_t irrigationFinishState = 0;
	uint8_t drainFinishState = 0;
	uint8_t valve1OpenState = 0;
	uint8_t valve2OpenState = 0;
	uint8_t valve3OpenState = 0;
	uint16_t gramsCounter = 0;
	uint32_t lastJoinAttemptMs = 0;
	bool joinInProgress = false;
	constexpr uint32_t LORAWAN_JOIN_RETRY_MS = 20000;
} // namespace

void lorawanSetup()
{
#ifdef LED_BUILTIN
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);
#endif
	if (LORA_PIN_RF_SW >= 0)
	{
		pinMode(LORA_PIN_RF_SW, OUTPUT);
		digitalWrite(LORA_PIN_RF_SW, HIGH);
	}

	hwConfig.CHIP_TYPE = SX1262_CHIP;
	hwConfig.PIN_LORA_RESET = LORA_PIN_RESET;
	hwConfig.PIN_LORA_NSS = LORA_PIN_NSS;
	hwConfig.PIN_LORA_SCLK = LORA_PIN_SCLK;
	hwConfig.PIN_LORA_MISO = LORA_PIN_MISO;
	hwConfig.PIN_LORA_DIO_1 = LORA_PIN_DIO1;
	hwConfig.PIN_LORA_BUSY = LORA_PIN_BUSY;
	hwConfig.PIN_LORA_MOSI = LORA_PIN_MOSI;
	hwConfig.RADIO_TXEN = LORA_RADIO_TXEN;
	hwConfig.RADIO_RXEN = LORA_RADIO_RXEN;
	hwConfig.USE_DIO2_ANT_SWITCH = true;
	hwConfig.USE_DIO3_TCXO = true;
	hwConfig.USE_DIO3_ANT_SWITCH = false;

	uint32_t err_code = lora_hardware_init(hwConfig);
	if (err_code != 0)
	{
		Serial.printf("lora_hardware_init failed - %lu\n", err_code);
	}

	err_code = timers_init();
	if (err_code != 0)
	{
		Serial.printf("timers_init failed - %lu\n", err_code);
	}

	fillDevEuiFromMac(nodeDeviceEUI);
	Serial.print("DevEUI derivado de la MAC: ");
	printDevEui(nodeDeviceEUI);
	char devEuiHex[17];
	formatDevEuiHex(devEuiHex);

	lmh_setDevEui(nodeDeviceEUI);
	lmh_setAppEui(nodeAppEUI);
	lmh_setAppKey(nodeAppKey);

	err_code = lmh_init(&lora_callbacks, lora_param_init, true, CLASS_A, currentRegion);
	if (err_code != 0)
	{
		Serial.printf("lmh_init failed - %lu\n", err_code);
	}

#if defined(REGION_US915) || defined(REGION_AU915)
	if (!lmh_setSubBandChannels(1))
	{
		Serial.println("lmh_setSubBandChannels failed. Wrong sub band requested?");
	}
#endif

	attemptJoin();
}

void lorawanLoop()
{
	if (lmh_join_status_get() != LMH_SET && !joinInProgress)
	{
		const uint32_t now = millis();
		if (lastJoinAttemptMs == 0 || (now - lastJoinAttemptMs) >= LORAWAN_JOIN_RETRY_MS)
		{
			attemptJoin();
		}
	}
}

void lorawanSetCommandHandler(LoraCommandHandler handler)
{
	commandHandler = handler;
}

void lorawanSetFinishStates(uint8_t irrigationState, uint8_t drainState)
{
	irrigationFinishState = irrigationState;
	drainFinishState = drainState;
}

void lorawanSetValveStates(uint8_t v1Open, uint8_t v2Open, uint8_t v3Open)
{
	valve1OpenState = v1Open ? 1 : 0;
	valve2OpenState = v2Open ? 1 : 0;
	valve3OpenState = v3Open ? 1 : 0;
}

void lorawanSetValve1State(uint8_t v1Open)
{
	valve1OpenState = v1Open ? 1 : 0;
}

void lorawanSetValve2State(uint8_t v2Open)
{
	valve2OpenState = v2Open ? 1 : 0;
}

void lorawanSetValve3State(uint8_t v3Open)
{
	valve3OpenState = v3Open ? 1 : 0;
}

void lorawan_join_failed_handler(void)
{
	Serial.println("OTAA join failed. Revisa DevEUI/AppEUI/AppKey y cobertura.");
	joinInProgress = false;
	lastJoinAttemptMs = millis();
}

void lorawan_has_joined_handler(void)
{
	Serial.println("Network joined (OTAA).");
	joinInProgress = false;
	lmh_class_request(CLASS_C);

	char devEuiHex[17];
	formatDevEuiHex(devEuiHex);

	uint32_t send_delay = LORAWAN_APP_TX_DUTYCYCLE + (rand() % APP_TX_DUTYCYCLE_RND);
	TimerSetValue(&appTimer, send_delay);
	TimerStart(&appTimer);
}

void lorawan_rx_handler(lmh_app_data_t *app_data)
{
	Serial.printf("RX on port %d, size:%d, rssi:%d, snr:%d -> ",
								app_data->port, app_data->buffsize, app_data->rssi, app_data->snr);
	for (uint8_t i = 0; i < app_data->buffsize; i++)
	{
		Serial.printf("%02X", app_data->buffer[i]);
	}
	Serial.println();

	String payload;
	payload.reserve(app_data->buffsize + 1);
	for (uint8_t i = 0; i < app_data->buffsize; ++i)
	{
		payload += static_cast<char>(app_data->buffer[i]);
	}
	payload.trim();

	Serial.println("Payload as string: " + payload);

	if (payload.length() > 0)
	{
		if (!extractCommandPayload(payload))
		{
			return;
		}

		LoraCommand cmd;
		if (parseLoraCommand(payload, cmd))
		{
			Serial.printf("Comando LoRa recibido: %s", commandName(cmd.type));
			if (cmd.type == LoraCommandType::Calibrate1 || cmd.type == LoraCommandType::Calibrate2 || cmd.type == LoraCommandType::SetTolerance)
			{
				Serial.printf(" (%.2f)", cmd.value);
			}
			Serial.println();

			if (commandHandler)
			{
				commandHandler(cmd);
			}
		}
		else
		{
			Serial.println("Payload LoRa no reconocido como comando: " + payload);
		}
	}
}

void lorawan_confirm_class_handler(DeviceClass_t Class)
{
	Serial.printf("Switch to class %c done\n", "ABC"[Class]);
	m_lora_app_data.buffsize = 0;
	m_lora_app_data.port = LORAWAN_APP_PORT;
	lmh_send(&m_lora_app_data, LMH_UNCONFIRMED_MSG);
}

void tx_lora_periodic_handler(void)
{
	uint32_t send_delay = LORAWAN_APP_TX_DUTYCYCLE + (rand() % APP_TX_DUTYCYCLE_RND);
	TimerSetValue(&appTimer, send_delay);
	TimerStart(&appTimer);
	Serial.println("Sending frame");
	sendGramsFrame();
}

uint32_t timers_init(void)
{
	appTimer.timerNum = 3;
	TimerInit(&appTimer, tx_lora_periodic_handler);
	return 0;
}

void sendGramsFrame()
{
	if (lmh_join_status_get() != LMH_SET)
	{
		Serial.println("LoRaWAN not joined, skip grams send");
		return;
	}

	buildGramsPayload(true);
	lmh_error_status error = lmh_send(&m_lora_app_data, LMH_UNCONFIRMED_MSG);
	LoadCells &loadCells = LoadCells::instance();
	const int32_t s1 = static_cast<int32_t>(lroundf(loadCells.read(1)));
	const int32_t s2 = static_cast<int32_t>(lroundf(loadCells.read(2)));
	uint8_t count1 = loadCells.recentReadingsCount(1);
	if (count1 > LoadCells::LORAWAN_HISTORY_SIZE)
	{
		count1 = LoadCells::LORAWAN_HISTORY_SIZE;
	}
	uint8_t count2 = loadCells.recentReadingsCount(2);
	if (count2 > LoadCells::LORAWAN_HISTORY_SIZE)
	{
		count2 = LoadCells::LORAWAN_HISTORY_SIZE;
	}
	Serial.printf("lmh_send grams result %d cnt=%u s1=%ld s2=%ld ir=%u dr=%u c1=%u c2=%u sz=%u\n",
								error,
								gramsCounter,
								static_cast<long>(s1),
								static_cast<long>(s2),
								static_cast<unsigned>(irrigationFinishState),
								static_cast<unsigned>(drainFinishState),
								static_cast<unsigned>(count1),
								static_cast<unsigned>(count2),
								static_cast<unsigned>(m_lora_app_data.buffsize));

#ifdef LED_BUILTIN
	digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
#endif
}

void buildGramsPayload(bool increaseCounter)
{
	if (increaseCounter)
	{
		++gramsCounter;
	}

	LoadCells &loadCells = LoadCells::instance();

	const int32_t s1 = static_cast<int32_t>(lroundf(loadCells.read(1)));
	const int32_t s2 = static_cast<int32_t>(lroundf(loadCells.read(2)));
	const uint32_t u1 = static_cast<uint32_t>(s1);
	const uint32_t u2 = static_cast<uint32_t>(s2);

	m_lora_app_data.buffer[0] = (gramsCounter >> 8) & 0xFF;
	m_lora_app_data.buffer[1] = gramsCounter & 0xFF;
	m_lora_app_data.buffer[2] = (u1 >> 24) & 0xFF;
	m_lora_app_data.buffer[3] = (u1 >> 16) & 0xFF;
	m_lora_app_data.buffer[4] = (u1 >> 8) & 0xFF;
	m_lora_app_data.buffer[5] = u1 & 0xFF;
	m_lora_app_data.buffer[6] = (u2 >> 24) & 0xFF;
	m_lora_app_data.buffer[7] = (u2 >> 16) & 0xFF;
	m_lora_app_data.buffer[8] = (u2 >> 8) & 0xFF;
	m_lora_app_data.buffer[9] = u2 & 0xFF;

	const bool ipOk = (WiFi.status() == WL_CONNECTED);
	IPAddress ip = ipOk ? WiFi.localIP() : IPAddress(0, 0, 0, 0);
	m_lora_app_data.buffer[10] = ip[0];
	m_lora_app_data.buffer[11] = ip[1];
	m_lora_app_data.buffer[12] = ip[2];
	m_lora_app_data.buffer[13] = ip[3];
	m_lora_app_data.buffer[14] = ipOk ? 1 : 0;
	m_lora_app_data.buffer[15] = irrigationFinishState;
	m_lora_app_data.buffer[16] = drainFinishState;
	m_lora_app_data.buffer[17] = Config_Get().tolerance;
	m_lora_app_data.buffer[18] = valve1OpenState;
	m_lora_app_data.buffer[19] = valve2OpenState;
	m_lora_app_data.buffer[20] = valve3OpenState;

	uint8_t cursor = 21;
	uint8_t count1 = loadCells.recentReadingsCount(1);
	if (count1 > LoadCells::LORAWAN_HISTORY_SIZE)
	{
		count1 = LoadCells::LORAWAN_HISTORY_SIZE;
	}
	m_lora_app_data.buffer[cursor++] = count1;
	for (uint8_t i = 0; i < count1; ++i)
	{
		const int32_t v = static_cast<int32_t>(loadCells.recentReading(1, i));
		const uint32_t uv = static_cast<uint32_t>(v);
		m_lora_app_data.buffer[cursor++] = (uv >> 24) & 0xFF;
		m_lora_app_data.buffer[cursor++] = (uv >> 16) & 0xFF;
		m_lora_app_data.buffer[cursor++] = (uv >> 8) & 0xFF;
		m_lora_app_data.buffer[cursor++] = uv & 0xFF;
	}

	uint8_t count2 = loadCells.recentReadingsCount(2);
	if (count2 > LoadCells::LORAWAN_HISTORY_SIZE)
	{
		count2 = LoadCells::LORAWAN_HISTORY_SIZE;
	}
	m_lora_app_data.buffer[cursor++] = count2;
	for (uint8_t i = 0; i < count2; ++i)
	{
		const int32_t v = static_cast<int32_t>(loadCells.recentReading(2, i));
		const uint32_t uv = static_cast<uint32_t>(v);
		m_lora_app_data.buffer[cursor++] = (uv >> 24) & 0xFF;
		m_lora_app_data.buffer[cursor++] = (uv >> 16) & 0xFF;
		m_lora_app_data.buffer[cursor++] = (uv >> 8) & 0xFF;
		m_lora_app_data.buffer[cursor++] = uv & 0xFF;
	}

	m_lora_app_data.buffsize = cursor;
	m_lora_app_data.port = LORAWAN_APP_PORT;
}

void fillDevEuiFromMac(uint8_t *devEui)
{
	uint8_t mac[6];
	esp_efuse_mac_get_default(mac);

	mac[0] ^= 0x02;
	devEui[0] = mac[0];
	devEui[1] = mac[1];
	devEui[2] = mac[2];
	devEui[3] = 0xFF;
	devEui[4] = 0xFE;
	devEui[5] = mac[3];
	devEui[6] = mac[4];
	devEui[7] = mac[5];
}

void printDevEui(const uint8_t *devEui)
{
	for (int i = 0; i < 8; i++)
	{
		Serial.printf("%02X", devEui[i]);
	}
	Serial.println();
}

int16_t readInternalTempC()
{
#if CONFIG_IDF_TARGET_ESP32
	extern "C" uint8_t temprature_sens_read(void);
	int rawF = temprature_sens_read();
	float celsius = (rawF - 32) / 1.8f;
	return static_cast<int16_t>(celsius * 100);
#else
	return 0;
#endif
}

void formatDevEuiHex(char *out)
{
	for (int i = 0; i < 8; i++)
	{
		snprintf(out + i * 2, 3, "%02X", nodeDeviceEUI[i]);
	}
	out[16] = '\0';
}

void attemptJoin()
{
	Serial.println("Joining LoRaWAN network (OTAA)...");
	lastJoinAttemptMs = millis();
	joinInProgress = true;
	lmh_join();
}

bool extractCommandPayload(String &payload)
{
	payload.trim();
	if (payload.length() == 0)
	{
		return false;
	}

	const int firstColon = payload.indexOf(':');
	if (firstColon > 0)
	{
		const int secondColon = payload.indexOf(':', firstColon + 1);
		if (secondColon > firstColon + 1 && secondColon + 1 < static_cast<int>(payload.length()))
		{
			String commandPart = payload.substring(secondColon + 1);
			commandPart.trim();
			if (commandPart.length() > 0)
			{
				payload = commandPart;
			}
		}
	}

	const int slashPos = payload.indexOf('/');
	if (slashPos > 0)
	{
		String idStr = payload.substring(0, slashPos);
		String cmdStr = payload.substring(slashPos + 1);
		idStr.trim();
		cmdStr.trim();
		bool allDigits = true;
		for (size_t i = 0; i < idStr.length(); ++i)
		{
			if (idStr[i] < '0' || idStr[i] > '9')
			{
				allDigits = false;
				break;
			}
		}
		if (allDigits && idStr.length() > 0)
		{
			const long cmdId = idStr.toInt();
			if (lastLoraCommandId == cmdId)
			{
				Serial.printf("LoRa comando duplicado (ID %ld), descartado\n", cmdId);
				return false;
			}
			lastLoraCommandId = cmdId;
			payload = cmdStr;
			return payload.length() > 0;
		}
		Serial.println("ID de comando LoRa invalido, se procesa sin filtro: " + idStr);
		payload = cmdStr;
		return payload.length() > 0;
	}

	return true;
}

bool parseLoraCommand(const String &payload, LoraCommand &out)
{
	String cmd = payload;
	cmd.trim();
	if (cmd.length() == 0)
	{
		return false;
	}

	String upper = cmd;
	upper.toUpperCase();

	if (upper == "TARE_1")
	{
		out.type = LoraCommandType::Tare1;
		return true;
	}
	if (upper == "TARE_2")
	{
		out.type = LoraCommandType::Tare2;
		return true;
	}
	if (upper == "TARE_BOTH")
	{
		out.type = LoraCommandType::TareBoth;
		return true;
	}
	if (upper.startsWith("TOLERANCE-"))
	{
		const String valueStr = cmd.substring(strlen("TOLERANCE-"));
		if (valueStr.length() == 0)
		{
			return false;
		}

		for (size_t i = 0; i < valueStr.length(); ++i)
		{
			if (valueStr[i] < '0' || valueStr[i] > '9')
			{
				return false;
			}
		}

		out.type = LoraCommandType::SetTolerance;
		out.value = valueStr.toFloat();
		return true;
	}
	if (upper == "IRRIGATION_FINISH")
	{
		out.type = LoraCommandType::IrrigationFinish;
		return true;
	}
	if (upper == "DRAIN_FINISH")
	{
		out.type = LoraCommandType::DrainFinish;
		return true;
	}
	if (upper == "RESET_FINISH_STATES" || upper == "RESET_FINISH")
	{
		out.type = LoraCommandType::ResetFinishStates;
		return true;
	}
	if (upper == "OPEN_V1")
	{
		out.type = LoraCommandType::OpenV1;
		return true;
	}
	if (upper == "CLOSE_V1")
	{
		out.type = LoraCommandType::CloseV1;
		return true;
	}
	if (upper == "OPEN_V2")
	{
		out.type = LoraCommandType::OpenV2;
		return true;
	}
	if (upper == "CLOSE_V2")
	{
		out.type = LoraCommandType::CloseV2;
		return true;
	}
	if (upper == "OPEN_V3")
	{
		out.type = LoraCommandType::OpenV3;
		return true;
	}
	if (upper == "CLOSE_V3")
	{
		out.type = LoraCommandType::CloseV3;
		return true;
	}
	if (upper.startsWith("CALIBRATE_1-"))
	{
		char *end = nullptr;
		const char *start = cmd.c_str() + strlen("CALIBRATE_1-");
		float grams = strtof(start, &end);
		if (end != start)
		{
			out.type = LoraCommandType::Calibrate1;
			out.value = grams;
			return true;
		}
	}
	if (upper.startsWith("CALIBRATE_2-"))
	{
		char *end = nullptr;
		const char *start = cmd.c_str() + strlen("CALIBRATE_2-");
		float grams = strtof(start, &end);
		if (end != start)
		{
			out.type = LoraCommandType::Calibrate2;
			out.value = grams;
			return true;
		}
	}

	out.type = LoraCommandType::Unknown;
	return false;
}

const char *commandName(LoraCommandType type)
{
	switch (type)
	{
	case LoraCommandType::Tare1:
		return "TARE_1";
	case LoraCommandType::Tare2:
		return "TARE_2";
	case LoraCommandType::TareBoth:
		return "TARE_BOTH";
	case LoraCommandType::Calibrate1:
		return "CALIBRATE_1";
	case LoraCommandType::Calibrate2:
		return "CALIBRATE_2";
	case LoraCommandType::SetTolerance:
		return "TOLERANCE";
	case LoraCommandType::IrrigationFinish:
		return "IRRIGATION_FINISH";
	case LoraCommandType::DrainFinish:
		return "DRAIN_FINISH";
	case LoraCommandType::ResetFinishStates:
		return "RESET_FINISH_STATES";
	case LoraCommandType::OpenV1:
		return "OPEN_V1";
	case LoraCommandType::CloseV1:
		return "CLOSE_V1";
	case LoraCommandType::OpenV2:
		return "OPEN_V2";
	case LoraCommandType::CloseV2:
		return "CLOSE_V2";
	case LoraCommandType::OpenV3:
		return "OPEN_V3";
	case LoraCommandType::CloseV3:
		return "CLOSE_V3";
	default:
		return "UNKNOWN";
	}
}
