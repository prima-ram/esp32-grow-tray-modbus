#include "WifiAp.h"

WifiAp::WifiAp(const String &WIFI_SSID, const String &WIFI_PASS)
{
	ssid_ = WIFI_SSID;
	pass_ = WIFI_PASS;
	lastCheck = 0;
	state = State::Idle;
	connectStart = 0;
}

bool WifiAp::connect()
{
	// Start or advance a connection attempt without blocking the main loop
	if (connected && WiFi.status() == WL_CONNECTED)
	{
		localIp = WiFi.localIP();
		Serial.println("Conected with ip" + WiFi.localIP());
		state = State::Connected;
		return true;
	}

	if (state == State::Idle)
	{
		WiFi.mode(WIFI_STA);
		WiFi.setAutoReconnect(true);
		WiFi.persistent(false);
		WiFi.begin(ssid_, pass_);
		connectStart = millis();
		state = State::Connecting;
	}

	if (state == State::Connecting)
	{
		if (WiFi.status() == WL_CONNECTED)
		{
			connected = true;
			localIp = WiFi.localIP();
			state = State::Connected;
			Serial.print("WiFi connected. IP: ");
			Serial.println(localIp);
			return true;
		}

		if (millis() - connectStart >= CONNECT_TIMEOUT_MS)
		{
			WiFi.disconnect(true);
			state = State::Idle; // retry later in loop()
		}
	}

	return connected;
}

void WifiAp::disconnect()
{
	if (!connected)
		return;

	WiFi.disconnect(true);
	WiFi.mode(WIFI_OFF);
	localIp = IPAddress(0, 0, 0, 0);
	connected = false;
	state = State::Idle;
}

IPAddress WifiAp::ip() const
{
	if (!isConnected())
		return IPAddress(0, 0, 0, 0);
	return localIp;
}

bool WifiAp::isConnected() const
{
	return WiFi.status() == WL_CONNECTED;
}

void WifiAp::loop()
{
	const unsigned long now = millis();

	if (state == State::Connected)
	{
		if (!isConnected())
		{
			connected = false;
			state = State::Idle;
			localIp = IPAddress(0, 0, 0, 0);
		}
		return;
	}

	if (state == State::Connecting)
	{
		connect(); // advance current attempt (non-blocking)
		return;
	}

	// Idle: start a new attempt every RETRY_DELAY_MS
	if (now - lastCheck >= RETRY_DELAY_MS)
	{
		lastCheck = now;
		connect();
	}
}
