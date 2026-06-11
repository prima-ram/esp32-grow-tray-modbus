#include "Config.h"
#include <Preferences.h>

// Configuration object
static Config g_config;

static const char NAME_KEY[] = "grow_tray";
static const char MAGIC_KEY[] = "magic";
static const char OFFSET_1_KEY[] = "offset_1";
static const char OFFSET_2_KEY[] = "offset_2";
static const char FACTOR_1_KEY[] = "factor_1";
static const char FACTOR_2_KEY[] = "factor_2";
static const char TOLERANCE_KEY[] = "tolerance";

// Flash config
Preferences prefs;

String macToString(const uint8_t mac[6])
{
	char buf[18];
	snprintf(buf, sizeof(buf),
					 "%02X:%02X:%02X:%02X:%02X:%02X",
					 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	return String(buf);
}

void Config_Load()
{
	if (!prefs.begin(NAME_KEY, false))
	{
		Serial.println("Error abriendo NVS");
		return;
	}

	uint32_t magic = prefs.getUInt(MAGIC_KEY, 0);

	if (magic != CONFIG_MAGIC)
	{
		// No valid config: set defaults
		g_config.magic = CONFIG_MAGIC;
		g_config.offset1 = 0;
		g_config.factor1 = 1.0f;
		g_config.offset2 = 0;
		g_config.factor2 = 1.0f;
		g_config.tolerance = 50;

		// Persist defaults once
		prefs.putUInt(MAGIC_KEY, g_config.magic);
		prefs.putLong(OFFSET_1_KEY, g_config.offset1);
		prefs.putFloat(FACTOR_1_KEY, g_config.factor1);
		prefs.putLong(OFFSET_2_KEY, g_config.offset2);
		prefs.putFloat(FACTOR_2_KEY, g_config.factor2);
		prefs.putUChar(TOLERANCE_KEY, g_config.tolerance);
	}
	else
	{
		g_config.magic = magic;
		g_config.offset1 = prefs.getLong(OFFSET_1_KEY, 0);
		g_config.factor1 = prefs.getFloat(FACTOR_1_KEY, 1.0f);
		g_config.offset2 = prefs.getLong(OFFSET_2_KEY, 0);
		g_config.factor2 = prefs.getFloat(FACTOR_2_KEY, 1.0f);
		g_config.tolerance = prefs.getUChar(TOLERANCE_KEY, 50);

		bool needsSave = false;

		if (isnan(g_config.factor1) || g_config.factor1 < 1)
		{
			g_config.factor1 = 1.0f;
			needsSave = true;
		}

		if (isnan(g_config.factor2) || g_config.factor2 < 1)
		{
			g_config.factor2 = 1.0f;
			needsSave = true;
		}

		if (g_config.tolerance > 250)
		{
			g_config.tolerance = 50;
			needsSave = true;
		}

		if (needsSave)
		{
			prefs.putFloat(FACTOR_1_KEY, g_config.factor1);
			prefs.putFloat(FACTOR_2_KEY, g_config.factor2);
			prefs.putUChar(TOLERANCE_KEY, g_config.tolerance);
		}
	}

	uint8_t mac[6];
	esp_efuse_mac_get_default(mac);

	g_config.mac = macToString(mac);

	prefs.end();
}

Config &Config_Get()
{
	return g_config;
}

void Config_Save()
{
	if (!prefs.begin(NAME_KEY, false))
	{
		Serial.println("Error abriendo NVS");
		return;
	}

	prefs.putUInt(MAGIC_KEY, g_config.magic);
	prefs.putLong(OFFSET_1_KEY, g_config.offset1);
	prefs.putFloat(FACTOR_1_KEY, g_config.factor1);
	prefs.putLong(OFFSET_2_KEY, g_config.offset2);
	prefs.putFloat(FACTOR_2_KEY, g_config.factor2);
	prefs.putUChar(TOLERANCE_KEY, g_config.tolerance);
	prefs.end();
}
