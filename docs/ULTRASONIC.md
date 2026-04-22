# Ultrasonic Sensor (HC-SR04) Wiring & Notes

- VCC: connect to 5V (or 3.3V only if your module supports it). Do NOT power a 5V-only module with 3.3V.
- GND: common ground between ESP32 and sensor.
- TRIG: connect to ESP32 digital output pin (default in `src/main.cpp`: GPIO5).
- ECHO: connect to ESP32 input pin (default in `src/main.cpp`: GPIO18). If your module outputs 5V on ECHO, use a level shifter or a resistor divider to bring the signal to 3.3V.
- Keep pings at a reasonable rate (recommended 50–100 ms between pings).
- Serial output format emitted by the example code:

```
ULTRASONIC: 123 cm
ULTRASONIC: TIMEOUT
```

Safety tips:
- Avoid using flash/strapping pins for TRIG/ECHO (GPIO6–GPIO11 and be cautious with 0/2/15 at boot).
- Add a small 0.1 µF bypass capacitor across sensor Vcc/GND if you observe noisy readings.

If you prefer not to use the NewPing library, you can implement a blocking `pulseIn()` approach instead; ask and I can add that variant.