# Touchlogo-Cap1188

This program can be uploaded to an ESP8266 to create a touch sensor which can detect swipes left, right and touches in a dependable way.

## Getting started

Add a CAP1188 chip to the I2C bus of the ESP8266 (on a Wemos D1 mini: D2 = SDA, D1 = SCL). In our setup 5 touch sensing channels (1-5) are connected to 5
line pads on a pcb. When sliding your hand over these pads the touch direction can be detected.

The firmware was written in platform.io.
