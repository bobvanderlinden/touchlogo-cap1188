#include <Wire.h>
#include <SPI.h>
#include <Adafruit_CAP1188.h>

Adafruit_CAP1188 cap = Adafruit_CAP1188();

void setup() {
  Serial.begin(9600);
  Serial.println("CAP1188 test!");

  if (!cap.begin(43)) {
    Serial.println("CAP1188 not found");
    while (1);
  }
  Serial.println("CAP1188 found!");

  cap.writeRegister(0x00,0b11000000);
  cap.writeRegister(0x71,0xff);
  cap.writeRegister(0x72,0xff);
  cap.writeRegister(0x73,0xff);
  cap.writeRegister(0x81,0b10101010);//Set led breath or pulse.
  cap.writeRegister(0x82,0b00000010);
}

void loop() {
  uint8_t touched = cap.touched();

  if (touched == 0) {
    // No touch detected
    return;
  }
  
  for (uint8_t i=0; i<8; i++) {
    if (touched & (1 << i)) {
      Serial.print("C"); Serial.print(i+1); Serial.print("\t");
    }
  }
  Serial.println();
  delay(50);
}