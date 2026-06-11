#include "Utils.h"
#include "Config.h"

String createStatusTopic(uint8_t sensorId)
{
	const Config cfg = Config_Get();
	const String topic = "grow_tray/" + cfg.mac + "/" + String(sensorId) + "/action";
	return topic;
}

String createReadingTopic(uint8_t sensorId, String measure)
{
	const Config cfg = Config_Get();
	const String topic = "raw/" + cfg.mac + "/grow_tray/" + String(sensorId) + "/" + measure.c_str();
	return topic;
}

String createTareTopic(uint8_t sensorId)
{
	const Config cfg = Config_Get();
	const String topic = "command/" + cfg.mac + "/grow_tray/" + String(sensorId);
	return topic;
}