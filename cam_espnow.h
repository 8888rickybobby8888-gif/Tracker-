/*
  cam_espnow.h - sends the ESP32's camera frames by ESP-NOW (through the same bridge as the lidar).   UNTESTED: written without the hardware.

  Only needed for follow.py --onboard bridge --lidar-serial <port>  (the follower's OWN camera). In the normal external-camera mode the ESP32's
  camera is not used at all.

  Use it together with lidar_udp.h compiled with  #define LIDAR_USE_ESPNOW 1  (that file starts ESP-NOW and the broadcast peer):
    1. #include "cam_espnow.h"  after lidar_udp.h in your sketch.
    2. Initialise the camera as the CameraWebServer example does, but with:   config.pixel_format = PIXFORMAT_JPEG;
         config.frame_size = FRAMESIZE_QVGA;   // 320x240. Try FRAMESIZE_QQVGA (160x120) if frames are too big
         config.jpeg_quality = 20;             // 10 = best/biggest ... 63 = worst/smallest. 20-30 keeps a QVGA frame around 4-8 KB
         config.fb_count = 1;
    3. In loop(), about every 100 ms:   camEspnowSend();      (and keep calling lidarPoll() often)

  Budget (estimates, not measured): ESP-NOW at its default 1 Mbit/s carries roughly 400-500 packets a second of ~200 bytes. A 6 KB frame is
  32 packets, so expect about 5-10 frames a second at QVGA, less if the lidar is also sending. A frame with a lost packet is dropped (broadcast has
  no retransmit). The USB serial line to the computer (921600 baud, about 90 KB/s) must carry the lot: lower the frame size or quality if it chokes,
  or raise the baud on both ends.
*/
#pragma once
#include <esp_camera.h>
#include <esp_now.h>

#define CAM_CHUNK_BYTES 190          // JPEG bytes per packet: 6 byte header + 190 = 196, under the 200 byte limit of the serial framing

static uint16_t camFrameId = 0;

// Packet: 'C' 'M' | frame id uint16 | chunk index uint8 | chunk count uint8 | up to CAM_CHUNK_BYTES of JPEG   (see lidar_rx.py)
inline void camEspnowSend() {
  static const uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;
  if (fb->format != PIXFORMAT_JPEG) { esp_camera_fb_return(fb); return; }
  const int total = (fb->len + CAM_CHUNK_BYTES - 1) / CAM_CHUNK_BYTES;
  if (total < 1 || total > 255) { esp_camera_fb_return(fb); return; }   // too big for the chunk count: lower the quality or frame size
  uint8_t buf[6 + CAM_CHUNK_BYTES];
  buf[0] = 'C';
  buf[1] = 'M';
  memcpy(buf + 2, &camFrameId, 2);
  buf[5] = (uint8_t)total;
  for (int i = 0; i < total; i++) {
    int n = (int)fb->len - i * CAM_CHUNK_BYTES;
    if (n > CAM_CHUNK_BYTES) n = CAM_CHUNK_BYTES;
    buf[4] = (uint8_t)i;
    memcpy(buf + 6, fb->buf + i * CAM_CHUNK_BYTES, n);
    for (int tries = 0; tries < 20; tries++) {                           // ESP-NOW's send queue is small: wait a moment if it is full
      if (esp_now_send(bcast, buf, 6 + n) == ESP_OK) break;
      delayMicroseconds(500);
    }
    delayMicroseconds(1500);                                             // pace the packets (about 600 a second at most)
  }
  camFrameId++;
  esp_camera_fb_return(fb);
}
