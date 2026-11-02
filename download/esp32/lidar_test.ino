/*
  lidar_test.ino - bring-up test for an 8x8 ST time-of-flight sensor (VL53L5CX) on an ESP32.
  UNTESTED: written without the hardware or a compiler. Treat it as a starting point.

  Library: "SparkFun VL53L5CX Arduino Library" (Library Manager).
  Your board is Pololu's VL53L7CX carrier. Pololu says the L7CX API appears functionally
  identical to the L5CX one and suggests this same SparkFun library for it. (Not verified by me.)

  Wiring:
    sensor VIN -> 5 V (or 3.3 V; see below)     GND -> GND (shared with the ESP32)
    SDA -> SDA_PIN            SCL -> SCL_PIN
    LPn, INT, I2C_RST can stay unconnected (the carrier pulls LPn up and I2C_RST down).
    VIN accepts 2.5-5.5 V. The sensor draws ~100 mA (peaks ~150 mA), so power it from a 5 V
    supply pin rather than the ESP32 board's 3.3 V pin if you can. I2C is level-shifted to VIN.
    Remove the protective liner over the sensor if one is fitted.
  Pick two FREE pins. Avoid the camera's pins, GPIO 12/0 (boot strapping), GPIO 6-11 (flash)
  and 16/17 (PSRAM on WROVER). If you use the SD card in 1-bit mode it uses 2, 14 and 15.

  Serial monitor: 115200 baud.
*/
#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h>

#define SDA_PIN 13        // CHANGE to free pins that your board exposes
#define SCL_PIN 32

SparkFun_VL53L5CX imager;
VL53L5CX_ResultsData data;
const int WIDTH = 8;

void scanI2C() {
  Serial.println("I2C scan:");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  device at 0x%02X\n", a);
      found++;
    }
  }
  if (!found) Serial.println("  nothing found: check wiring, power and LPn");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  scanI2C();                                  // the VL53 family appears at 0x29

  Serial.println("Starting sensor (firmware upload can take a few seconds)...");
  if (!imager.begin()) {
    Serial.println("Sensor not found. Halting.");
    while (true) delay(1000);
  }
  imager.setResolution(WIDTH * WIDTH);        // 8x8
  imager.setRangingFrequency(15);             // Hz
  imager.startRanging();
  Serial.println("Ranging. Wave a hand in front of the LEFT side and see which columns react.");
}

void loop() {
  if (imager.isDataReady() && imager.getRangingData(&data)) {
    // Same print order as the SparkFun example, so it looks like the view from the sensor.
    for (int y = 0; y <= WIDTH * (WIDTH - 1); y += WIDTH) {
      for (int x = WIDTH - 1; x >= 0; x--) {
        Serial.print('\t');
        Serial.print(data.distance_mm[x + y]);
      }
      Serial.println();
    }
    Serial.println();
  }
  delay(5);
}
