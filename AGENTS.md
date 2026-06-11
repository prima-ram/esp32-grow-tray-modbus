## Project Context

- Board: Seeed Studio XIAO ESP32S3.
- PlatformIO env: `env:xiao_esp32s3_modbus`, Arduino framework.
- Purpose: grow-tray controller with two load cells, Modbus RTU, pressure sensing, and I2C valve control. MQTT, Wi-Fi, OTA, and LoRaWAN are not used in the active build.
- External communication: Modbus RTU only. Serial is local bench diagnostics/commands only and must not be documented as an external interface.
- Modbus RTU: `Serial1` at 9600 baud, `SERIAL_8N1`, RX GPIO8, TX GPIO9, slave id `2`.
- Pressure sensor: analog pin `A8`; voltage uses `adc * 3.62 / 4095.0`; current uses `voltage / 100.0 * 1000.0`; pressure uses `(mA - 4.0) * 10.0 / 16.0` bar.
- I2C motor driver: Grove I2C Motor Driver (TB6612FNG), addresses `0x0F` (inlet) and `0x0E` (drain).
- Valves are maintained-coil valves, not latching valves. Keep the driver state energized or stopped according to `Valve::Type`; do not implement pulse-and-stop behavior unless the hardware changes. Any continuously energized valve is automatically de-energized after 10 minutes as a safety cutoff; the reported open state follows the de-energized hardware state for the valve type.

## Key Modules

- `src/main.cpp`: application entrypoint for Config, load cells, hoppers, Modbus, pressure, and local serial commands.
- `ModbusOrders`: Modbus RTU register map, command processing, pressure registers, and valve state synchronization.
- `LoadCells`: tare/calibrate/density, serial command parsing, non-blocking HX711 refresh.
- `WaterInletHopper`: valve 1 control and irrigation-finish state machine.
- `DrainWaterHopper`: valve 2/3 control and drain-finish state machine.
- `Valve`: I2C driver wrapper for TB6612FNG commands and maintained coil state.
- `Config`: persistent offsets, factors, densities, tolerance, and MAC.

## Modbus Register Map

Register ranges:

- `1-99`: primary process readings.
- `100-199`: action, calibration, and control registers.
- `200-299`: valve command/current-state registers (200-202) and diagnostics (203-208).

All `write trigger` registers use value `1` to execute the action and the firmware resets them to `0` after processing. Register `109` is not a trigger: it stores the finish tolerance in grams, and writing a new value updates `Config.tolerance` persistently.

| Register | Direction     | Meaning                     | Scale / Values                                                                 |
| -------- | ------------- | --------------------------- | ------------------------------------------------------------------------------ |
| 1        | read          | Irrigation weight           | grams, signed int16                                                            |
| 2        | read          | Drain weight                | grams, signed int16                                                            |
| 3        | read          | Calculated pressure         | centibar (`bar * 100`), signed int16                                           |
| 100      | write trigger | Tare cell 1                 | write `1`; firmware resets to `0`                                              |
| 101      | write trigger | Tare cell 2                 | write `1`; firmware resets to `0`                                              |
| 102      | write         | Calibration weight cell 1   | decigrams (`grams * 10`)                                                       |
| 103      | write trigger | Calibrate cell 1            | write `1`; firmware resets to `0`                                              |
| 104      | write         | Calibration weight cell 2   | decigrams (`grams * 10`)                                                       |
| 105      | write trigger | Calibrate cell 2            | write `1`; firmware resets to `0`                                              |
| 106      | write trigger | Finish irrigation           | write `1`; firmware resets to `0`                                              |
| 107      | write trigger | Finish drain                | write `1`; firmware resets to `0`                                              |
| 108      | write trigger | Reset device                | write `1`; firmware resets to `0`                                              |
| 109      | read/write    | Finish tolerance            | grams (0-255); write updates Config on change                                  |
| 200      | read/write    | V1 command/current state    | write `0` close or `1` open; read current state                                |
| 201      | read/write    | V2 command/current state    | write `0` close or `1` open; read current state                                |
| 202      | read/write    | V3 command/current state    | write `0` close or `1` open; read current state                                |
| 203      | read          | Pressure sensor voltage     | millivolts                                                                     |
| 204      | read          | Pressure sensor current     | `mA * 100`                                                                     |
| 205      | read          | Pressure ADC raw            | raw ADC count                                                                  |
| 206      | read          | Load cell 1 error           | `0` ok, `1` no valid reading yet                                               |
| 207      | read          | Load cell 2 error           | `0` ok, `1` no valid reading yet                                               |
| 208      | read          | Valve Modbus command status | `0` ok; `1` invalid V1 command; `2` invalid V2 command; `3` invalid V3 command |

## Communication Contracts

- Modbus RTU is the only external communication contract; keep the ordered register map unchanged.
- Local serial valve commands: `OPEN_V1`, `CLOSE_V1`, `OPEN_V2`, `CLOSE_V2`, `OPEN_V3`, `CLOSE_V3`. Serial valve commands must update the matching Modbus valve command register.
- Local serial diagnostics: `I2C_SCAN`.
- Load cell serial commands are handled by `LoadCells::processCommand()`.

## Coding Guidelines

- Comments in English; prefer ASCII unless existing content uses otherwise.
- Keep loops non-blocking; rely on `.loop()`/`.task()` calls for Modbus, hoppers, and load cells.
- When changing offsets, factors, or densities: use `Config_Get()` and persist with `Config_Save()`.
- Validate numeric inputs before applying them to sensors.
- Preserve serial command names.

## Build And Tooling

- Build: `pio run -e xiao_esp32s3_modbus`.
- Flash over USB: `pio run -e xiao_esp32s3_modbus -t upload`.
- Serial monitor: `pio device monitor -e xiao_esp32s3_modbus` at 115200 baud, `LF` EOL.
- Dependencies (PlatformIO): HX711_ADC, modbus-esp8266, Preferences, Grove I2C Motor Driver TB6612FNG.
- NEVER BUILD the firmware unless user give permissions

## Notes

- Maintained-coil valves must keep the required energized/stopped state. Do not add pulse timers for normal open/close commands. The only timeout behavior is the 10-minute safety cutoff for continuous energization; after cutoff, Modbus command/current-state registers `200`-`202` must mirror the real de-energized state.
- Do not reintroduce separate valve-state registers unless the Modbus contract changes; `200`-`202` are both command and current-state registers.
- `src/main_old.cpp` and `src/main2.cpp` are legacy references and are excluded from the active build.
- `src/lorawan.cpp` and `src/WifiAp.cpp` are inactive legacy/reference files and are excluded from the active build.
