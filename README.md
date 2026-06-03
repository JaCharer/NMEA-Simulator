# NMEA & AIS Simulator v3.2

An ESP32-based tool that generates NMEA 0183, NMEA 2000, and Signal K data streams. It is used for testing navigation software (such as OpenCPN) and verifying marine instrument displays.

## Features
- **Physics Model**: Simulates hull speed, wind influence (leeway/drift), and engine effects.
- **Skipper AI**: A waypoint-following function with basic propulsion management.
- **AIS Targets**: Simulates several vessels (ferry, tanker, fishing boat) with basic collision avoidance logic (ARPA).
- **Protocol Support**: 
    - **NMEA 0183**: Port `10110` (TCP/UDP).
    - **NMEA 2000**: CAN bus support via a transceiver (e.g., SN65HVD230).
    - **Signal K**: Data via WebSocket and support for HTTP API requests.
    - **Web Interface**: Control panel and configuration accessible via browser.
- **Network Management**: Supports Access Point (AP) and Station (STA) modes with NTP time synchronization.
- **Hardware Support**: Optimized for TTGO T-Display but compatible with any generic ESP32.
- **Architecture**: Uses FreeRTOS to separate tasks (physics, network, display) for stable 10Hz updates.

## Hardware Configuration
This project is designed to be flexible regarding hardware:
- **Standard ESP32**: Works on any ESP32 development board. If you don't use a display, simply disable the flags in `platformio.ini`.
- **Display & Buttons**: By default, it supports the ST7789 LCD and buttons found on the TTGO T-Display.
- **Custom Pins**:
    - **CAN Bus**: Pins can be redefined in `platformio.ini` using `-D ESP32_CAN_TX=GPIO_NUM_X` and `-D ESP32_CAN_RX=GPIO_NUM_X`.
    - **Buttons**: If `USE_BUTTONS` is active, pins are defined in `LCD_Buttons.h` (defaulting to GPIO 0 and 35 for TTGO).
- **Build Flags**:
    - Comment out `-D USE_LCD` to compile without display support.
    - Comment out `-D USE_BUTTONS` to disable physical button interaction.

## Protocols and Ports
- **NMEA 0183**: Port `10110` (TCP/UDP).
- **Signal K**: Port `80` (Endpoints: `/signalk/v1/stream` and API).
- **Web Dashboard**: Port `80` (HTTP).

## Usage
1. **Build**: Use PlatformIO and select hardware flags (`USE_LCD`, `USE_BUTTONS`).
2. **Files**: Upload the contents of the `data` folder to the ESP32 memory (LittleFS).
3. **Connection**: Connect to WiFi `Jacht_Symulator` (password: `password123`). Configure settings at `http://192.168.4.1`.

## Control Logic
- **Manual Mode**: Direct course/engine control via Web UI or physical buttons.
- **Navi Mode (Autopilot)**: The simulator accepts external corrections from OpenCPN (NMEA RMB / Signal K PUT) only when the `Navi` flag is enabled via the Web UI **and Skipper AI is inactive.**
- **Skipper AI**: When activated via Web, the AI takes full control of the vessel, ignoring external corrections until manual intervention or deactivation. **It prioritizes safe navigation and optimal propulsion.**

## Project Structure
- `main.cpp`: Main loop and task orchestration.
- `yacht_physics.cpp`: Vessel movement logic (wind, current, engine).
- `N2K_Handler.cpp`: NMEA 2000 stack handling and PGN transmission.
- `ais.cpp`: Vessel simulation and AIS encoder.
- `NMEA0183.cpp`: Generator for NMEA 0183 text sentences.
- `SignalK.cpp`: JSON generation and WebSocket handling.
- `WebServer.cpp`: HTTP server and dashboard hosting.
- `LCD_Buttons.cpp`: LCD display and physical button management.
- `runSkipperAI.cpp`: Autopilot decision logic.
- `data/`: Web user interface files.

## Requirements
- ESP32 development board
- PlatformIO IDE
- Libraries: `TFT_eSPI`, `AsyncTCP`, `ESPAsyncWebServer`, `ArduinoJson @ ^7.0.0`.

## Troubleshooting
- **TCP Connection Issues**: The simulator supports up to 3 simultaneous TCP clients. If all slots are full, the oldest connection is dropped to make room.
- **Incorrect Time**: The system syncs with NTP. Without internet access, it defaults to a fallback date (2026-01-01).
- **Factory Reset**: Hold both buttons on the TTGO for 5 seconds to clear WiFi settings and return to AP mode.

## License
MIT License
