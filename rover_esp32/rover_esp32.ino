/*
  rover_esp32.ino - the WHOLE program for the ESP32 on the rover, in one sketch (no merging into CameraWebServer):

      camera  -> MJPEG stream   http://<ip>:81/stream     (follow.py --camera 1 --onboard http://<ip>:81/stream)
      lidar   -> UDP broadcast  port 4210                 (follow.py listens on it by default)

  UNTESTED: written without the hardware or the Arduino toolchain. Expect to fix a pin or a library detail on the first compile.

  Folder: this file and lidar_udp.h must sit together in a folder called rover_esp32 (the folder name must match the .ino name).

  Arduino IDE setup
    1. Boards Manager: install "esp32 by Espressif Systems".
       (File > Preferences > Additional boards manager URLs:
        https://espressif.github.io/arduino-esp32/package_esp32_index.json )
    2. Library Manager: install "SparkFun VL53L5CX Arduino Library".
    3. Tools > Board > esp32 > "AI Thinker ESP32-CAM"   (or "ESP32 Wrover Module" for a WROVER board)
       Tools > PSRAM: Enabled          Tools > Partition Scheme: "Huge APP (3MB No OTA/1MB SPIFFS)"
       Tools > Upload Speed: 115200 (slower is more reliable)
    4. Edit WIFI_SSID / WIFI_PASS below (a 2.4 GHz network: the ESP32 cannot use 5 GHz) and the camera model.
    5. Upload, open Serial Monitor at 115200: it prints the address, e.g.  Camera stream: http://192.168.4.23:81/stream
    The IDE cannot see the board? See "Board not detected" in README.md (cable, driver, adapter, boot mode).

  Wiring of the lidar (VL53L7CX): VIN -> 5V, GND -> GND, SDA -> LIDAR_SDA, SCL -> LIDAR_SCL (pins below). Remove the liner on the sensor.
*/
#include <WiFi.h>
#include "esp_camera.h"
#include "esp_http_server.h"

// ============================== EDIT THESE ==============================
const char* WIFI_SSID = "YourHotspot";
const char* WIFI_PASS = "YourPassword";

#define CAMERA_MODEL_AI_THINKER        // the common "ESP32-CAM". For an ESP32-WROVER-KIT style board use CAMERA_MODEL_WROVER_KIT instead
// #define CAMERA_MODEL_WROVER_KIT

// Lidar I2C pins: two FREE pins. AI-Thinker: 14 and 15 are free if you do not use the SD card. Avoid 0, 2, 12 (boot), 6-11 (flash), 16 (PSRAM).
#define LIDAR_SDA 14
#define LIDAR_SCL 15
#define LIDAR_WIRE Wire1               // the camera driver keeps I2C port 0 (pins 26/27) to itself
// ========================================================================

#include "lidar_udp.h"

#if defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#elif defined(CAMERA_MODEL_WROVER_KIT)
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 21
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 19
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 5
#define Y2_GPIO_NUM 4
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#else
#error "Pick a camera model above (CAMERA_MODEL_AI_THINKER or CAMERA_MODEL_WROVER_KIT)"
#endif

#define STREAM_BOUNDARY "123456789000000000000987654321"

static bool cameraOk = false;

static bool cameraInit() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA;          // 640x480; follow.py's field of view setting assumes the picture is this wide or any other: only the ratio matters
  config.jpeg_quality = 12;                   // 10 (best) .. 63 (smallest)
  config.fb_count = psramFound() ? 2 : 1;
  config.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;      // always the newest picture, never a queue of old ones
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init FAILED (0x%x). Check the camera model, the ribbon cable and the 5 V supply.\n", err);
    return false;
  }
  return true;
}

static esp_err_t streamHandler(httpd_req_t* req) {
  esp_err_t res = httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=" STREAM_BOUNDARY);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  char part[80];
  while (true) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { res = ESP_FAIL; break; }
    size_t hlen = snprintf(part, sizeof(part), "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", (unsigned)fb->len);
    res = httpd_resp_send_chunk(req, "\r\n--" STREAM_BOUNDARY "\r\n", strlen("\r\n--" STREAM_BOUNDARY "\r\n"));
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, part, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    if (res != ESP_OK) break;                 // the computer closed the stream
  }
  return res;
}

static esp_err_t indexHandler(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_sendstr(req, "rover_esp32 is running. The camera stream is at /stream on this port (81).");
}

static void startServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 81;
  config.ctrl_port = 32769;
  httpd_uri_t index_uri = {"/", HTTP_GET, indexHandler, NULL};
  httpd_uri_t stream_uri = {"/stream", HTTP_GET, streamHandler, NULL};
  httpd_handle_t server = NULL;
  if (httpd_start(&server, &config) == ESP_OK) {
    httpd_register_uri_handler(server, &index_uri);
    httpd_register_uri_handler(server, &stream_uri);
  } else {
    Serial.println("Could not start the camera web server");
  }
}

static void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                       // power saving adds latency
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("Connecting to WiFi '%s' ", WIFI_SSID);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
    if (millis() - t0 > 20000) {              // try again from scratch
      Serial.println("\nStill not connected: check the name and password, and that it is a 2.4 GHz network.");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      t0 = millis();
    }
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  Serial.println("\nrover_esp32 starting");
  cameraOk = cameraInit();
  connectWifi();
  if (cameraOk) startServer();
  lidarBegin();                               // after WiFi is up (it uses the same network)
  Serial.printf("Camera stream: http://%s:81/stream\n", WiFi.localIP().toString().c_str());
  Serial.printf("Lidar: UDP broadcast on port %d\n", LIDAR_UDP_PORT);
}

void loop() {
  lidarPoll();
  delay(1);
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 15000) {          // repeat the address, in case the Serial Monitor was opened late
    lastPrint = millis();
    Serial.printf("Camera stream: http://%s:81/stream\n", WiFi.localIP().toString().c_str());
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi lost: reconnecting");
    WiFi.reconnect();
    delay(2000);
  }
}
