# NMEA & AIS Bridge Simulator v3.2

An advanced ESP32-based maritime bridge simulator generating realistic NMEA 0183, AIS, and **comprehensive Signal K** data streams. Designed for testing navigation software (OpenCPN, qtVlm) and calibrating marine instruments.

## Features
- **Realistic Physics Engine**: Simulates hull speed (~9.5 kn), **motorsailing, realistic leeway (drift)**, and wind influence on course and speed. **Includes NaN rollback for physics stability.**
- **Virtual Skipper AI**: An autonomous waypoint-following system with intelligent engine/sail management **and dynamic decision-making based on wind conditions.**
- **Dynamic AIS Targets**: Simulates multiple vessels (Ferry, Tanker, Fishing boat) with built-in **ARPA/COLREG collision avoidance logic and realistic behavior.**
- **Multi-Protocol Support**: 
    - **NMEA 0183**: Port `10110` (TCP/UDP).
    - **Signal K**: Port `80` (WebSocket & **HTTP API for external control**).
    - **Web Interface**: Real-time Dashboard and Configuration.
- **Network Management**: Supports Access Point (AP) and Station (STA) modes with automatic fallback and NTP time synchronization. **Includes robust TCP client management with connection recycling.**
- **Hardware Optimized**: Dedicated support for TTGO T-Display (LCD TFT, control buttons, factory reset **via physical buttons**).
- **Performance**: **Dual-core FreeRTOS architecture ensuring stable 10Hz simulation updates** even under high network load, with **dedicated tasks for Signal K and GUI.**

## Protocols and Ports
- **NMEA 0183**: Port `10110` (TCP/UDP).
- **Signal K**: Port `80` (Endpoint: `/signalk` for discovery, `/signalk/v1/stream` for WebSocket, `/signalk/v1/api/...` for HTTP PUT).
- **Web Dashboard**: Port `80` (HTTP).

## Usage
1. **Build**: Use PlatformIO. Ensure the correct build flags are set in `platformio.ini` for your hardware (`USE_LCD`, `USE_BUTTONS`).
2. **Upload Filesystem**: Upload the contents of the `data` folder to the ESP32 (LittleFS) to enable the Web UI.
3. **Connect**: Connect to the WiFi `Jacht_Symulator` (password: `password123`).
4. **Configure**: Access `http://192.168.4.1` to set simulation parameters, logging levels (0-3), or to connect the simulator to your home WiFi. **The web interface provides real-time feedback and control.**
5. **Navigate**: In OpenCPN, add a network connection to the device IP on port `10110`.

## Control Logic
- **Manual Mode**: Direct course/engine control via Web UI or physical buttons.
- **Navi Mode (Autopilot)**: The simulator accepts external corrections from OpenCPN (NMEA RMB / Signal K PUT) only when the `Navi` flag is enabled via the Web UI **and Skipper AI is inactive.**
- **Skipper AI**: When activated via Web, the AI takes full control of the vessel, ignoring external corrections until manual intervention or deactivation. **It prioritizes safe navigation and optimal propulsion.**

## Project Structure
- `main.cpp`: Main orchestrator, physics engine, network handling, and FreeRTOS tasks.
- `ais.cpp`: **AIS target behavior logic, ARPA calculations, and efficient bitwise AIS encoders.**
- `NMEA0183.cpp`: **Generates various NMEA 0183 sentences with checksums.**
- `SignalK.cpp`: **Handles Signal K data generation and WebSocket communication.**
- `WebServer.cpp`: **Manages AsyncWebServer, WebSockets (dashboard and Signal K), and HTTP API endpoints.**
- `yacht_physics.cpp`: **Detailed yacht physics simulation including wind, current, and engine effects.**
- `LCD_Buttons.cpp`: **Manages LCD display updates and physical button interactions.**
- `runSkipperAI.cpp`: **Implements the Skipper AI's decision-making for waypoint navigation and propulsion.**
- `data/`: Web interface files (HTML/JS/CSS).

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
