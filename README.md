# ESP32_CAMFaceRecognitionSecurityAdminModeAccsessPoint
# Explanation of the Code

This code demonstrates a facial recognition system using the ESP32-CAM module, a relay-controlled door, and WebSockets communication. The system allows for capturing, enrolling, and recognizing faces. It also provides functionality to stream the camera feed and interact with a client via WebSockets to control the door based on facial recognition.

### Libraries and Definitions

```cpp
#include <ArduinoWebsockets.h>
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "camera_index.h"
#include "Arduino.h"
#include "fd_forward.h"
#include "fr_forward.h"
#include "fr_flash.h"
```
These libraries include the necessary functionality for the ESP32-CAM to handle HTTP server requests, WebSockets communication, camera operation, and face detection and recognition.

```cpp
const char* ap_ssid = "ESP32-CAM";
const char* ap_password = "123456789";
```
Defines the Access Point (AP) SSID and password that the ESP32-CAM will broadcast, allowing clients to connect.

```cpp
#define ENROLL_CONFIRM_TIMES 5
#define FACE_ID_SAVE_NUMBER 100
```
Defines constants for the face enrollment process. `ENROLL_CONFIRM_TIMES` specifies how many samples are required for face enrollment, and `FACE_ID_SAVE_NUMBER` defines the maximum number of faces that can be stored.

```cpp
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"
```
Specifies the model of the camera module and includes the necessary pin configuration for the ESP32-CAM.

```cpp
WebsocketsServer socket_server;
```
Creates a WebSocket server to handle incoming WebSocket connections.

```cpp
camera_fb_t * fb = NULL;
```
A pointer to store the camera frame buffer.

```cpp
long current_millis;
long last_detected_millis = 0;
```
Variables to manage time intervals, primarily used to handle the door relay's timed unlocking and face detection timing.

```cpp
#define relay_pin 2
unsigned long door_opened_millis = 0;
long interval = 5000;
bool face_recognised = false;
```
Defines the pin controlling the relay (door lock), the interval for door locking after opening, and flags for face recognition.

### Setup Function

```cpp
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
```
Initializes the serial communication for debugging.

```cpp
  digitalWrite(relay_pin, LOW);
  pinMode(relay_pin, OUTPUT);
```
Configures the relay pin for output, initially setting it to LOW (locked).

```cpp
  camera_config_t config;
  ...
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
```
Configures and initializes the camera based on the ESP32-CAM module's pin configuration.

```cpp
  WiFi.softAP(ap_ssid, ap_password);
  IPAddress IP = WiFi.softAPIP();
```
Starts the ESP32 in Access Point mode with the specified SSID and password. The device will broadcast this AP, and clients can connect to it.

```cpp
  app_httpserver_init();
  app_facenet_main();
  socket_server.listen(82);
```
Starts the HTTP server and initializes the facial recognition system. The WebSocket server listens on port 82 for incoming WebSocket connections.

### HTTP Server Setup

```cpp
static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
  return httpd_resp_send(req, (const char *)index_ov2640_html_gz, index_ov2640_html_gz_len);
}
```
Handles HTTP requests to the root endpoint (`/`) by sending an HTML page compressed with gzip encoding.

```cpp
httpd_uri_t index_uri = {
  .uri       = "/",
  .method    = HTTP_GET,
  .handler   = index_handler,
  .user_ctx  = NULL
};
```
Registers the `index_handler` to handle GET requests to the root URI (`/`).

```cpp
void app_httpserver_init () {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    Serial.println("HTTP server started");
    httpd_register_uri_handler(camera_httpd, &index_uri);
  }
}
```
Starts the HTTP server and registers the index URI handler.

### Facial Recognition and Enrollment

```cpp
void app_facenet_main() {
  face_id_name_init(&st_face_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);
  aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
  read_face_id_from_flash_with_name(&st_face_list);
}
```
Initializes the facial recognition system, including the list to store face IDs and loading previously saved face data from flash memory.

```cpp
static inline int do_enrollment(face_id_name_list *face_list, dl_matrix3d_t *new_id) {
  int left_sample_face = enroll_face_id_to_flash_with_name(face_list, new_id, st_name.enroll_name);
  return left_sample_face;
}
```
Handles the enrollment of a new face by saving multiple samples to flash memory.

### WebSocket Message Handling

```cpp
void handle_message(WebsocketsClient &client, WebsocketsMessage msg) {
  if (msg.data() == "stream") {
    g_state = START_STREAM;
    client.send("STREAMING");
  }
  if (msg.data() == "detect") {
    g_state = START_DETECT;
    client.send("DETECTING");
  }
  ...
}
```
Handles incoming WebSocket messages. The state is changed based on the message, and appropriate actions (such as streaming, detecting, capturing, or recognizing) are taken.

```cpp
void open_door(WebsocketsClient &client) {
  if (digitalRead(relay_pin) == LOW) {
    digitalWrite(relay_pin, HIGH);
    client.send("door_open");
    door_opened_millis = millis();
  }
}
```
Unlocks the door if it is locked and sends a "door_open" message to the WebSocket client.

### Main Loop

```cpp
void loop() {
  auto client = socket_server.accept();
  client.onMessage(handle_message);
  ...
  while (client.available()) {
    client.poll();
    fb = esp_camera_fb_get();
    ...
    if (g_state == START_DETECT || g_state == START_ENROLL || g_state == START_RECOGNITION) {
      out_res.net_boxes = face_detect(image_matrix, &mtmn_config);
      if (out_res.net_boxes) {
        if (align_face(out_res.net_boxes, image_matrix, aligned_face) == ESP_OK) {
          out_res.face_id = get_face_id(aligned_face);
          ...
          if (g_state == START_RECOGNITION  && (st_face_list.count > 0)) {
            face_id_node *f = recognize_face_with_name(&st_face_list, out_res.face_id);
            if (f) {
              open_door(client);
              client.send("DOOR OPEN");
            } else {
              client.send("FACE NOT RECOGNISED");
            }
          }
        }
      }
    }
    client.sendBinary((const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    fb = NULL;
  }
}
```
Main loop to continuously check for incoming messages and process camera frames. If a face is detected and recognized, the door is opened. The system also supports capturing and enrolling new faces for recognition.

### Summary

This project integrates an ESP32-CAM with facial recognition and WebSockets to control a relay for unlocking a door. It supports the following features:
1. **Streaming**: Send a live camera feed via WebSocket.
2. **Face Detection**: Detect faces in the camera feed.
3. **Enrollment**: Enroll a new face by capturing multiple samples.
4. **Recognition**: Recognize stored faces and unlock the door if a match is found.
5. **WebSocket Control**: Allow the user to control the device through WebSocket messages, including streaming, detecting, enrolling, and recognizing faces.

