#pragma once

#include <stdint.h>

// Arranque y bucle principal de la pila LoRaWAN (OTAA).
void lorawanSetup();
void lorawanLoop();

enum class LoraCommandType
{
	Tare1,
	Tare2,
	TareBoth,
	Calibrate1,
	Calibrate2,
	SetTolerance,
	IrrigationFinish,
	DrainFinish,
	ResetFinishStates,
	OpenV1,
	CloseV1,
	OpenV2,
	CloseV2,
	OpenV3,
	CloseV3,
	Unknown
};

struct LoraCommand
{
	LoraCommandType type = LoraCommandType::Unknown;
	float value = 0.0f; // gramos para calibracion; 0 en el resto
};

// Registra un callback opcional para comandos recibidos por LoRaWAN (downlink).
using LoraCommandHandler = void (*)(const LoraCommand &cmd);
void lorawanSetCommandHandler(LoraCommandHandler handler);

// Exporta estados de secuencias finish para incluirlos en telemetria uplink.
void lorawanSetFinishStates(uint8_t irrigationState, uint8_t drainState);
void lorawanSetValveStates(uint8_t v1Open, uint8_t v2Open, uint8_t v3Open);
void lorawanSetValve1State(uint8_t v1Open);
void lorawanSetValve2State(uint8_t v2Open);
void lorawanSetValve3State(uint8_t v3Open);
