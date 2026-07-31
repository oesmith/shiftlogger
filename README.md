# Shiftlogger

Shiftlogger is a combined shift light / telemetry logger project for my
Caterham Seven track car.

It's built on a [ResjaCAN-ESP32](https://github.com/MagnusThome/RejsaCAN-ESP32)
board, interfaces to the ECU via the ESP32 CAN peripheral, and logs data to the
built-in microSD card slot. The shift lights are a 'stick' of 8x WS2812 RGB
LEDs.
