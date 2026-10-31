/*
  espnow_bridge.ino - the USB end of the ESP-NOW lidar link.   UNTESTED: written without the hardware or a compiler.

  Flash this to the SECOND ESP32 (any ESP32 board). Plug it into the computer by USB. It listens for the lidar ESP32's ESP-NOW packets
  (lidar_udp.h with  #define LIDAR_USE_ESPNOW 1) and forwards each one down the USB serial line, framed so the computer can resync:

      0xA5 0x5A | length (1 byte) | the "LD" packet exactly as lidar_rx.py describes | CRC-8 of the packet (polynomial 0x07)

  On the computer:   python follow.py --camera 1 --lidar-serial COM5          (or /dev/ttyUSB0; baud 921600 by default)

  Notes
  - ESPNOW_CHANNEL must equal LIDAR_ESPNOW_CHANNEL in lidar_udp.h.
  - The serial baud must equal --lidar-baud (default 921600). The lidar is about 137 bytes x 15 Hz = 2 KB/s, so even 115200 would do:
    if your USB-serial chip is flaky at 921600, change both ends to 460800 or 115200.
  - Do not print anything else to Serial here: every byte on that line is parsed by the computer.
  - The callback signature changed in ESP32 Arduino core 3.x; both are handled below.
*/
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define ESPNOW_CHANNEL 1
#define SERIAL_BAUD 921600

static uint8_t crc8(const uint8_t *d, int n) {
  uint8_t crc = 0;
  for (int i = 0; i < n; i++) {
    crc ^= d[i];
    for (int b = 0; b < 8; b++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}

static void forward(const uint8_t *data, int len) {
  if (len <= 0 || len > 200) return;
  uint8_t head[3] = {0xA5, 0x5A, (uint8_t)len};
  Serial.write(head, 3);
  Serial.write(data, len);
  Serial.write(crc8(data, len));
}

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) { forward(data, len); }
#else
void onRecv(const uint8_t *mac, const uint8_t *data, int len) { forward(data, len); }
#endif

void setup() {
  Serial.begin(SERIAL_BAUD);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    while (true) { delay(1000); }                    // nothing sensible to print: the serial line is for lidar frames only
  }
  esp_now_register_recv_cb(onRecv);
}

void loop() {
  delay(1000);
}
