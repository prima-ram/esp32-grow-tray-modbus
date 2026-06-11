#pragma once

#include <Arduino.h>
#include <WiFi.h>

// Singleton to manage connection to an existing WiFi network
class WifiAp
{
public:
	WifiAp(const String &WIFI_SSID, const String &WIFI_PASS);

	// Connect to the configured WiFi network (non-blocking).
	bool connect();

	// Disconnect from WiFi
	void disconnect();

	// Get local IP after connection
	IPAddress ip() const;

	bool isConnected() const;

	// Must be called regularly in loop() to keep WIFI alive
	void loop();

private:
	const unsigned long CHECK_INTERVAL = 5000;
	unsigned long lastCheck;

	static const uint32_t CONNECT_TIMEOUT_MS = 10000; // wait per attempt
	static const uint32_t RETRY_DELAY_MS = 2000;			// wait before retry

	enum class State
	{
		Idle,
		Connecting,
		Connected
	};

	State state = State::Idle;
	unsigned long connectStart = 0;

	String ssid_;
	String pass_;
	bool connected = false;
	IPAddress localIp;
};
