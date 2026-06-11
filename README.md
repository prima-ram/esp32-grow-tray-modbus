# Bandejas ESP32 Grow-Tray Controller

Firmware for a Seeed Studio XIAO ESP32S3 grow-tray controller with two load cells, maintained-coil valves, Modbus RTU, and an analog pressure sensor.

Modbus RTU is the only external communication interface in the active firmware. Serial commands are kept for local bench diagnostics only.

## Quick Path
1. Build: `pio run -e xiao_esp32s3_modbus`.
2. Flash over USB: `pio run -e xiao_esp32s3_modbus -t upload`.
3. Monitor serial: `pio device monitor -e xiao_esp32s3_modbus`.

Serial monitor uses 115200 baud and `LF` line endings.

## Hardware And Runtime
| Area | Value |
|------|-------|
| Board | Seeed Studio XIAO ESP32S3 |
| Framework | Arduino via PlatformIO |
| Modbus | RTU slave id `2`, `Serial1`, 9600 baud, RX GPIO8, TX GPIO9 |
| Pressure input | `A8` |
| Inlet motor driver | Grove TB6612FNG at `0x0F` |
| Drain motor driver | Grove TB6612FNG at `0x0E` |

## Modbus Map

Register ranges are intentionally grouped so the PLC/SCADA can reason about the map without memorizing individual addresses:

- `1-99`: primary process readings.
- `100-199`: action and calibration triggers.
- `200-299`: valve command/current-state registers.
- `300+`: diagnostics, raw values, and errors.

| Register | Direction | Meaning | Scale / Values |
|----------|-----------|---------|----------------|
| 1 | read | Irrigation weight | grams, signed int16 |
| 2 | read | Drain weight | grams, signed int16 |
| 3 | read | Calculated pressure | centibar (`bar * 100`), signed int16 |
| 100 | write trigger | Tare cell 1 | write `5`; firmware resets to `0` |
| 101 | write trigger | Tare cell 2 | write `5`; firmware resets to `0` |
| 102 | write | Calibration weight cell 1 | decigrams (`grams * 10`) |
| 103 | write trigger | Calibrate cell 1 | write `1`; firmware resets to `0` |
| 104 | write | Calibration weight cell 2 | decigrams (`grams * 10`) |
| 105 | write trigger | Calibrate cell 2 | write `1`; firmware resets to `0` |
| 200 | read/write | V1 command/current state | write `0` close or `1` open; read current state |
| 201 | read/write | V2 command/current state | write `0` close or `1` open; read current state |
| 202 | read/write | V3 command/current state | write `0` close or `1` open; read current state |
| 300 | read | Pressure sensor voltage | millivolts |
| 301 | read | Pressure sensor current | `mA * 100` |
| 302 | read | Pressure ADC raw | raw ADC count |
| 303 | read | Load cell 1 error | `0` ok, `1` no valid reading yet |
| 304 | read | Load cell 2 error | `0` ok, `1` no valid reading yet |
| 305 | read | Valve Modbus command status | `0` ok; `1` invalid V1 command; `2` invalid V2 command; `3` invalid V3 command |

## Local Debug Commands
Serial valve commands are local bench controls only. They mirror the corresponding Modbus valve command register so register state stays coherent.

Serial valve commands:
- `OPEN_V1`, `CLOSE_V1`
- `OPEN_V2`, `CLOSE_V2`
- `OPEN_V3`, `CLOSE_V3`

Serial diagnostics:
- `I2C_SCAN`

## Valve Behavior
Valves are maintained-coil valves, not latching valves.

`Valve::Type::NormallyClosed` opens by energizing the driver and closes by stopping it. `Valve::Type::NormallyOpen` opens by stopping the driver and closes by energizing it. Do not add pulse-and-stop behavior for normal valve commands unless the hardware changes.

Any valve energized continuously for 10 minutes is automatically de-energized as a safety cutoff. After the cutoff, the firmware updates the valve state from the de-energized hardware state: normally closed valves report closed, and normally open valves report open. Modbus registers `200`-`202` mirror that current state, so no separate valve-state registers are used.

## Notes
- `src/main.cpp` is the active application entrypoint.
- `src/main_old.cpp` and `src/main2.cpp` are legacy references and are excluded from the build.
- `src/lorawan.cpp` and `src/WifiAp.cpp` remain as inactive legacy/reference files and are excluded from the active build.
