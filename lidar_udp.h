/*
  lidar_udp.h - reads the VL53L7CX rear lidar and sends it to the computer over WiFi (UDP).
  Add this file to the CameraWebServer sketch folder.   UNTESTED: written without the hardware or a compiler.

  In CameraWebServer.ino make three small changes:
    1. near the top:                     #include "lidar_udp.h"
    2. at the END of setup():            lidarBegin();      // after WiFi has connected
    3. in loop(), replace delay(10000):  lidarPoll(); delay(1);

  Library: "SparkFun VL53L5CX Arduino Library" (Pololu says it also works for the L7CX).

  Packet (little-endian), parsed by lidar_rx.py:
    'L','D'  | seq uint16 | zones uint8 | millis uint32 | zones x uint16 distance in mm (0xFFFF = no reading)

  Notes
  - Wiring is in lidar_test.ino (VIN to 5 V, GND shared, SDA/SCL to two free pins).
  - The camera driver also uses an I2C bus for its own control lines. This file uses Wire (I2C port 0) on
    different pins. If the camera or the lidar fails to start, that sharing is the first thing to check.
  - Packets are broadcast on the local network so you don't need the computer's IP. If your router blocks
    broadcast, set LIDAR_DEST to the computer's IP instead.
*/
#pragma once
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SparkFun_VL53L5CX_Library.h>

#define LIDAR_SDA 13            // CHANGE to free pins on your board
#define LIDAR_SCL 32
#define LIDAR_UDP_PORT 4210     // must match --lidar-port in follow.py
#define LIDAR_GRID 4            // 4 = 4x4 zones (faster), 8 = 8x8 zones. Must match GRID in evade.py
#define LIDAR_HZ 30             // I believe 4x4 allows up to 60 Hz and 8x8 up to 15 Hz

static SparkFun_VL53L5CX lidarDev;
static VL53L5CX_ResultsData lidarData;
static WiFiUDP lidarUdp;
static uint16_t lidarSeq = 0;
static bool lidarOk = false;

inline void lidarBegin() {
  Wire.begin(LIDAR_SDA, LIDAR_SCL);
  Wire.setClock(400000);
  Serial.println("Lidar: starting (firmware upload can take a few seconds)...");
  if (!lidarDev.begin(0x29, Wire)) {
    Serial.println("Lidar: NOT FOUND. Check wiring and power.");
    return;
  }
  lidarDev.setResolution(LIDAR_GRID * LIDAR_GRID);
  lidarDev.setRangingFrequency(LIDAR_HZ);
  lidarDev.startRanging();
  lidarOk = true;
  Serial.println("Lidar: ranging");
}

inline void lidarPoll() {
  if (!lidarOk || !lidarDev.isDataReady()) return;
  if (!lidarDev.getRangingData(&lidarData)) return;

  const int n = LIDAR_GRID * LIDAR_GRID;
  uint8_t buf[9 + 2 * 64];
  buf[0] = 'L';
  buf[1] = 'D';
  memcpy(buf + 2, &lidarSeq, 2);
  lidarSeq++;
  buf[4] = (uint8_t)n;
  uint32_t ms = millis();
  memcpy(buf + 5, &ms, 4);
  for (int i = 0; i < n; i++) {
    uint16_t d = 0xFFFF;
    uint8_t st = lidarData.target_status[i];
    // ST documents status 5 (and 9) as a valid range; everything else is treated as "no reading".
    if ((st == 5 || st == 9) && lidarData.distance_mm[i] > 0) d = (uint16_t)lidarData.distance_mm[i];
    memcpy(buf + 9 + 2 * i, &d, 2);
  }
  lidarUdp.beginPacket(WiFi.broadcastIP(), LIDAR_UDP_PORT);
  lidarUdp.write(buf, 9 + 2 * n);
  lidarUdp.endPacket();
}
