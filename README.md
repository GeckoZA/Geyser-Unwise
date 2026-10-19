# GeyserUnwise v3.1 – Smart Geyser Control

ESP-12F firmware: ADS1115 + NTC + relay, MQTT/HA, multi-page web UI.

## Modes

| Mode | Behaviour |
|------|-----------|
| **Off** | Heater forced off |
| **Auto** | Heat to setpoint; off; re-on at setpoint − hysteresis (default 5 °C). Respects schedule window. |
| **Boost** | Once-off heat to boost temp, then Auto. Optional soft-min (only if temp below threshold). |

## Features

- **Schedule** – enable/disable daily heat window (overnight OK). Outside window → Auto cannot heat; Boost still allowed.
- **Presets** – Home / Away / Eco setpoints (web + MQTT).
- **Runtime** – heater hours today / week / total (MQTT + Stats page).
- **Temp graph** – ~2 h history on Stats page (`/api/history`).
- **Sensor fault** – invalid/NAN for 3 min → force off + HA binary_sensor; never turn on while invalid.
- **Soft min boost** – optional °C threshold; boost skipped if already warmer.
- **Factory reset** – Update page button, or hold **FLASH (GPIO0)** 5 s at boot.
- **LED** – GPIO2 on when WiFi connected.
- **OTA** – ArduinoOTA + optional GitHub `firmware.bin` releases.

## Hardware

| Part | Pin |
|------|-----|
| Relay | GPIO14 |
| ADS1115 SDA/SCL | GPIO4 / GPIO5 |
| Built-in LED | GPIO2 |
| Factory reset | GPIO0 (FLASH) |

NTC: 3.3V — NTC — A0 — 10.98 kΩ — GND.

## Build

```bash
pio run -t upload
pio run -t uploadfs
```

## Web pages

| Path | Content |
|------|---------|
| `/` | Dial, Off/Auto/Boost, Home/Away/Eco |
| `/connections` | AP + MQTT |
| `/time` | Timezone |
| `/temperature` | Setpoints, hysteresis, min on/off, soft min boost |
| `/schedule` | Enable + on/off times |
| `/stats` | Runtime hours + temperature graph |
| `/update` | OTA + factory reset |

## MQTT (base `geyser/<chipid>/`)

State JSON includes: temperature, setpoint, boost_temp, mode, preset, heater, sensor_fault, schedule_active, runtime_*_h.

Commands: `cmd/setpoint`, `cmd/boost_temp`, `cmd/mode`, `cmd/preset` (`boost`/`none`/`home`/`away`/`eco`).
