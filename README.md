# NMEA & AIS Bridge Simulator

This project is a simulator for marine navigation systems, generating realistic NMEA 0183 and AIS data streams. It is designed for testing and demonstration of marine navigation software and hardware, such as OpenCPN and navigation instruments.

## Features
- Simulates vessel movement, wind, engine, and navigation parameters
- Generates NMEA 0183 and AIS messages
- Supports TCP, UDP, and serial output
- Web interface for configuration and monitoring
- Realistic physics and collision avoidance logic
- Multiple vessel (AIS target) simulation

## Usage
1. Flash the firmware to your ESP32 device using PlatformIO.
2. Connect to the device via WiFi (AP or STA mode).
3. Access the web interface for configuration (default: http://192.168.4.1 or device IP).
4. Connect your navigation software (e.g., OpenCPN) to the simulator via TCP/UDP port 10110.
5. Monitor NMEA/AIS data on the serial port or network.

## Requirements
- ESP32 development board
- PlatformIO

## Example Applications
- Testing navigation software (OpenCPN, qtVlm, etc.)
- Demonstrating marine electronics
- Educational purposes

## License
MIT License
