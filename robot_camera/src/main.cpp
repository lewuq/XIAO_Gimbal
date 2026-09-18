#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>
#include "dl_image_jpeg.hpp"
#include "esp_camera.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "human_face_detect.hpp"

static const char *TAG = "gimbal_recamera";
static SemaphoreHandle_t frame_mux, control_mux;
static std::vector<uint8_t> latest_jpeg;
static uint32_t latest_frame_id;
static float yaw = 180.0f, pitch = 90.0f;
static bool automatic = true, person_valid = false;
static float person_x0 = 0, person_y0 = 0, person_x1 = 0, person_y1 = 0, person_score = 0;
static uint16_t sequence;
static int last_uart_written;
static QueueHandle_t uart_tx_queue;
static bool telemetry_valid;
static float motor_yaw, motor_pitch, motor_yaw_target, motor_pitch_target;
static uint8_t motor_state;
static uint16_t telemetry_sequence;

struct __attribute__((packed)) Packet
{
  uint8_t a, b, version, flags;
  uint16_t seq;
  uint16_t yaw, pitch;
  uint16_t crc;
};
struct QueuedTarget
{
  float yaw;
  float pitch;
  uint8_t flags;
};
struct __attribute__((packed)) TelemetryPacket
{
  uint8_t a, b, version, flags;
  uint16_t seq;
  uint16_t yaw, pitch, yaw_target, pitch_target;
  uint16_t crc;
};
static uint16_t crc16(const uint8_t *p, size_t n)
{
  uint16_t c = 0xffff;
  while (n--)
  {
    c ^= (uint16_t)*p++ << 8;
    for (int i = 0; i < 8; i++)
      c = (c & 0x8000) ? (c << 1) ^ 0x1021 : c << 1;
  }
  return c;
}
static bool send_target(uint8_t request_flags = 0)
{
  QueuedTarget q{yaw, pitch,
    (uint8_t)(request_flags | (automatic ? 2 : 0) | (person_valid ? 4 : 0))};
  BaseType_t ok = request_flags ? xQueueSendToFront(uart_tx_queue, &q, 0) :
                                  xQueueSend(uart_tx_queue, &q, 0);
  if (ok != pdTRUE) ESP_LOGW(TAG, "UART TX queue full; flags=0x%02x", q.flags);
  return ok == pdTRUE;
}

static void uart_tx_task(void *)
{
  QueuedTarget q;
  for (;;)
  {
    if (xQueueReceive(uart_tx_queue, &q, portMAX_DELAY) != pdTRUE) continue;
    Packet p{0xaa, 0x55, 1, q.flags, ++sequence,
      (uint16_t)lroundf(q.yaw * 100), (uint16_t)lroundf(q.pitch * 100), 0};
    p.crc = crc16((uint8_t *)&p, sizeof(p) - 2);
    int written = uart_write_bytes(UART_NUM_1, &p, sizeof(p));
    esp_err_t done = written == (int)sizeof(p) ?
      uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(20)) : ESP_FAIL;
    last_uart_written = done == ESP_OK ? written : -1;
    if ((p.flags & 1U) || done != ESP_OK || (sequence % 20U) == 0U)
      ESP_LOGI(TAG, "UART TX seq=%u bytes=%d/%u done=%d flags=0x%02x yaw=%.1f pitch=%.1f",
               sequence, written, (unsigned)sizeof(p), done, p.flags,
               (double)q.yaw, (double)q.pitch);
  }
}

static void uart_rx_task(void *)
{
  uint8_t frame[sizeof(TelemetryPacket)];
  size_t used = 0;
  for (;;)
  {
    uint8_t byte;
    if (uart_read_bytes(UART_NUM_1, &byte, 1, pdMS_TO_TICKS(100)) != 1) continue;
    if (used == 0 && byte != 0x55) continue;
    if (used == 1 && byte != 0xaa) { used = byte == 0x55 ? 1 : 0; continue; }
    frame[used++] = byte;
    if (used != sizeof(frame)) continue;
    auto *p = reinterpret_cast<const TelemetryPacket *>(frame);
    uint16_t expected = crc16(frame, sizeof(frame) - 2);
    if (p->version == 1 && p->crc == expected &&
        xSemaphoreTake(control_mux, pdMS_TO_TICKS(10)))
    {
      telemetry_valid = true;
      telemetry_sequence = p->seq;
      motor_state = p->flags;
      motor_yaw = p->yaw / 100.0f;
      motor_pitch = p->pitch / 100.0f;
      motor_yaw_target = p->yaw_target / 100.0f;
      motor_pitch_target = p->pitch_target / 100.0f;
      xSemaphoreGive(control_mux);
      if ((p->seq % 20U) == 0U)
        ESP_LOGI(TAG, "UART RX seq=%u state=%u yaw=%.2f pitch=%.2f target=%.2f/%.2f",
                 p->seq, p->flags, (double)motor_yaw, (double)motor_pitch,
                 (double)motor_yaw_target, (double)motor_pitch_target);
    }
    else if (p->crc != expected)
      ESP_LOGW(TAG, "UART RX CRC got=0x%04x expected=0x%04x", p->crc, expected);
    used = 0;
  }
}

static esp_err_t camera_init()
{
  camera_config_t c{};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = 15;
  c.pin_d1 = 17;
  c.pin_d2 = 18;
  c.pin_d3 = 16;
  c.pin_d4 = 14;
  c.pin_d5 = 12;
  c.pin_d6 = 11;
  c.pin_d7 = 48;
  c.pin_xclk = 10;
  c.pin_pclk = 13;
  c.pin_vsync = 38;
  c.pin_href = 47;
  c.pin_sccb_sda = 40;
  c.pin_sccb_scl = 39;
  c.pin_pwdn = -1;
  c.pin_reset = -1;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = FRAMESIZE_QVGA;
  c.jpeg_quality = 12;
  c.fb_count = 2;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;
  return esp_camera_init(&c);
}
static void capture(void *)
{
  for (;;)
  {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb)
    {
      if (xSemaphoreTake(frame_mux, pdMS_TO_TICKS(30)))
      {
        latest_jpeg.assign(fb->buf, fb->buf + fb->len);
        ++latest_frame_id;
        xSemaphoreGive(frame_mux);
      }
      esp_camera_fb_return(fb);
    }
    vTaskDelay(pdMS_TO_TICKS(15));
  }
}
static void infer(void *)
{
  HumanFaceDetect detector(HumanFaceDetect::ESPDET_PICO_224_224_FACE, false);
  detector.set_score_thr(0.30f);
  std::vector<uint8_t> jpg;
  uint32_t cycles = 0;
  uint32_t processed_frame_id = 0;
  float filtered_cx = 160.0f, filtered_cy = 120.0f;
  bool filter_valid = false;
  ESP_LOGI(TAG, "Face detector started: ESPDet-Pico-224 S8 threshold=0.30");
  for (;;)
  {
    if (xSemaphoreTake(frame_mux, pdMS_TO_TICKS(50)))
    {
      if (latest_frame_id != processed_frame_id) {
        jpg = latest_jpeg;
        processed_frame_id = latest_frame_id;
      } else {
        jpg.clear();
      }
      xSemaphoreGive(frame_mux);
    }
    if (jpg.empty())
    {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    dl::image::jpeg_img_t jpeg{.data = jpg.data(), .data_len = jpg.size()};
    auto img = dl::image::sw_decode_jpeg(jpeg, dl::image::DL_IMAGE_PIX_TYPE_RGB888);
    if (!img.data) {
      ESP_LOGW(TAG, "JPEG decode failed");
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }
    int64_t infer_started = esp_timer_get_time();
    auto &ds = detector.run(img);
    uint32_t infer_ms = (uint32_t)((esp_timer_get_time() - infer_started) / 1000);
    bool found = false;
    float area = 0, cx = 0, cy = 0, bx0 = 0, by0 = 0, bx1 = 0, by1 = 0, score = 0;
    for (auto &d : ds)
    {
      float a = (d.box[2] - d.box[0]) * (d.box[3] - d.box[1]);
      if (a > area)
      {
        area = a;
        bx0 = d.box[0];
        by0 = d.box[1];
        bx1 = d.box[2];
        by1 = d.box[3];
        cx = (bx0 + bx1) * .5f;
        cy = (by0 + by1) * .5f;
        score = d.score;
        found = true;
      }
    }
    if (xSemaphoreTake(control_mux, pdMS_TO_TICKS(20)))
    {
      person_valid = found;
      person_x0 = bx0;
      person_y0 = by0;
      person_x1 = bx1;
      person_y1 = by1;
      person_score = score;
      if (automatic && found)
      {
        const float frame_cx = img.width * 0.5f;
        const float frame_cy = img.height * 0.5f;
        if (!filter_valid) {
          filtered_cx = cx;
          filtered_cy = cy;
          filter_valid = true;
        } else {
          /* Light smoothing suppresses detector jitter without adding a long
           * moving-average delay. */
          /* Detection already arrives about 230 ms late.  Heavy filtering made
           * the controller steer using an even older face position. */
          filtered_cx += 0.85f * (cx - filtered_cx);
          filtered_cy += 0.85f * (cy - filtered_cy);
        }
        float ex = (filtered_cx - frame_cx) / frame_cx;
        float ey = (filtered_cy - frame_cy) / frame_cy;
        bool moved = false;
        if (fabsf(ex) > .06f) {
          /* A face left of centre requires increasing physical yaw on this
           * gimbal.  The old '+' sign was positive feedback and pushed the
           * face out of frame.  Base corrections on measured angle to avoid
           * accumulating commands while the motor is still moving. */
          const float yaw_base = telemetry_valid ? motor_yaw : yaw;
          yaw = std::clamp(yaw_base - std::clamp(ex * 5.0f, -4.0f, 4.0f), 1.f, 344.f);
          moved = true;
        }
        if (fabsf(ey) > .07f) {
          /* Image Y grows downward; physical pitch-positive raises the camera. */
          const float pitch_base = telemetry_valid ? motor_pitch : pitch;
          pitch = std::clamp(pitch_base - std::clamp(ey * 4.0f, -3.0f, 3.0f), 1.f, 175.f);
          moved = true;
        }
        if (moved) {
          send_target();
          ESP_LOGI(TAG, "AUTO TRACK ex=%+.3f ey=%+.3f target=%.1f/%.1f motor=%.1f/%.1f",
                   (double)ex, (double)ey, (double)yaw, (double)pitch,
                   (double)motor_yaw, (double)motor_pitch);
        }
      } else if (!found) {
        filter_valid = false;
      }
      xSemaphoreGive(control_mux);
    }
    heap_caps_free(img.data);
    if ((++cycles % 10U) == 0U)
      ESP_LOGI(TAG, "FACE detections=%u face=%d score=%.2f infer=%ums box=[%.0f %.0f %.0f %.0f]", (unsigned)ds.size(), found, (double)score, infer_ms, (double)bx0, (double)by0, (double)bx1, (double)by1);
    /* Yield briefly to Wi-Fi/camera tasks; the previous fixed 120 ms delay
     * dominated end-to-end tracking latency. */
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static const char PAGE[] = R"HTML(<!doctype html><meta name=viewport content="width=device-width,initial-scale=1,user-scalable=no">
<style>body{margin:0;background:#111;color:#eee;text-align:center;font:15px sans-serif}h3{margin:8px}#v{position:relative;display:inline-block;width:100%;max-width:640px}img{display:block;width:100%}canvas{position:absolute;inset:0;width:100%;height:100%;pointer-events:none}#s{height:24px;line-height:24px;padding:0 5px;color:#7f7;white-space:nowrap;overflow:hidden;text-overflow:ellipsis;font:12px monospace}.bar{display:grid;grid-template-columns:45px 1fr 48px;gap:6px;align-items:center;max-width:430px;margin:7px auto;padding:0 10px}.bar input{width:100%}button{font-size:16px;padding:7px 11px;margin:3px}#j{margin:10px auto;width:160px;height:160px;border-radius:50%;background:#333;touch-action:none;position:relative}#k{position:absolute;left:57px;top:57px;width:46px;height:46px;border-radius:50%;background:#36c}</style>
<h3>Gimbal Cam</h3><div id=v><img id=cam><canvas id=box></canvas></div><div id=s>READY</div>
<p><button onclick="cmd('/mode?auto=1','AUTO')">AUTO</button><button onclick="cmd('/mode?auto=0','MAN')">MAN</button><button onclick="cmd('/center','CENTER')">CENTER</button><button onclick="cmd('/cal','CAL')">CAL</button></p>
<div class=bar><label>Yaw</label><input id=yr type=range min=1 max=344 step=.1 value=180><output id=yv>180.0</output></div>
<div class=bar><label>Pitch</label><input id=pr type=range min=1 max=175 step=.1 value=90><output id=pv>90.0</output></div>
<div id=j><div id=k></div></div>
<script>
let j=$('#j'),k=$('#k'),cam=$('#cam'),cv=$('#box'),s=$('#s'),yr=$('#yr'),pr=$('#pr'),yv=$('#yv'),pv=$('#pv'),drag=0,last=0,pending='',busy=0,slideTimer;
function $(q){return document.querySelector(q)}cam.src=`http://${location.hostname}:81/stream`;
function cmd(u,t){fetch(u).then(r=>r.text()).then(()=>s.textContent=t+' OK').catch(()=>s.textContent=t+' ERR')}
async function pump(){if(!pending){busy=0;return}busy=1;let u=pending;pending='';try{await fetch(u)}catch(e){s.textContent='CTRL ERR'}pump()}
function queue(u){pending=u;if(!busy)pump()}
function angles(){yv.value=(+yr.value).toFixed(1);pv.value=(+pr.value).toFixed(1);clearTimeout(slideTimer);slideTimer=setTimeout(()=>queue(`/angles?yaw=${yr.value}&pitch=${pr.value}`),100)}yr.oninput=angles;pr.oninput=angles;
function move(e){if(!drag)return;let now=Date.now();if(now-last<80)return;last=now;let r=j.getBoundingClientRect(),x=(e.clientX-r.left-80)/80,y=(e.clientY-r.top-80)/80,z=Math.max(1,Math.hypot(x,y));x/=z;y/=z;k.style.transform=`translate(${x*52}px,${y*52}px)`;queue(`/control?x=${x.toFixed(3)}&y=${y.toFixed(3)}`)}
j.onpointerdown=e=>{drag=1;last=0;move(e)};onpointermove=move;onpointerup=()=>{drag=0;k.style.transform=''};
async function stat(){try{let q=await(await fetch('/status')).json(),w=cam.clientWidth,h=cam.clientHeight,c=cv.getContext('2d');cv.width=w;cv.height=h;c.clearRect(0,0,w,h);let det=q.face?`F${Math.round(q.score*100)}`:'F--',act=q.rx?`${q.myaw.toFixed(1)}/${q.mpitch.toFixed(1)}`:'--/--';s.textContent=`${q.auto?'AUTO':'MAN'} ${det} CMD ${q.yaw.toFixed(1)}/${q.pitch.toFixed(1)} ACT ${act} S${q.state} T${q.seq} R${q.rxseq}`;if(!drag&&document.activeElement!==yr&&document.activeElement!==pr){yr.value=q.yaw;pr.value=q.pitch;yv.value=q.yaw.toFixed(1);pv.value=q.pitch.toFixed(1)}if(q.face){c.strokeStyle='#0f5';c.lineWidth=3;c.strokeRect(q.x0*w/320,q.y0*h/240,(q.x1-q.x0)*w/320,(q.y1-q.y0)*h/240);c.fillStyle='#0f5';c.font='bold 16px sans-serif';c.fillText('face '+Math.round(q.score*100)+'%',q.x0*w/320,Math.max(18,q.y0*h/240-4))}}catch(e){s.textContent='STATUS ERR'}setTimeout(stat,400)}stat();
</script>)HTML";
static esp_err_t root(httpd_req_t *r)
{
  httpd_resp_set_type(r, "text/html");
  return httpd_resp_send(r, PAGE, HTTPD_RESP_USE_STRLEN);
}
static esp_err_t status(httpd_req_t *r)
{
  char out[384];
  bool pv, au, tv;
  float x0, y0, x1, y1, sc, ya, pi, my, mp, myt, mpt;
  uint8_t ms;
  uint16_t seq, rxseq;
  if (xSemaphoreTake(control_mux, pdMS_TO_TICKS(20)))
  {
    pv = person_valid;
    au = automatic;
    x0 = person_x0;
    y0 = person_y0;
    x1 = person_x1;
    y1 = person_y1;
    sc = person_score;
    ya = yaw;
    pi = pitch;
    seq = sequence;
    tv = telemetry_valid;
    my = motor_yaw;
    mp = motor_pitch;
    myt = motor_yaw_target;
    mpt = motor_pitch_target;
    ms = motor_state;
    rxseq = telemetry_sequence;
    xSemaphoreGive(control_mux);
  }
  else
    return httpd_resp_send_err(r, HTTPD_500_INTERNAL_SERVER_ERROR, "busy");
  int n = snprintf(out, sizeof(out), "{\"face\":%s,\"auto\":%s,\"x0\":%.1f,\"y0\":%.1f,\"x1\":%.1f,\"y1\":%.1f,\"score\":%.3f,\"yaw\":%.1f,\"pitch\":%.1f,\"seq\":%u,\"txbytes\":%d,\"rx\":%s,\"rxseq\":%u,\"state\":%u,\"myaw\":%.1f,\"mpitch\":%.1f,\"myawt\":%.1f,\"mpitcht\":%.1f}", pv ? "true" : "false", au ? "true" : "false", (double)x0, (double)y0, (double)x1, (double)y1, (double)sc, (double)ya, (double)pi, seq, last_uart_written, tv ? "true" : "false", rxseq, ms, (double)my, (double)mp, (double)myt, (double)mpt);
  httpd_resp_set_type(r, "application/json");
  return httpd_resp_send(r, out, n);
}
static esp_err_t stream(httpd_req_t *r)
{
  httpd_resp_set_type(r, "multipart/x-mixed-replace;boundary=frame");
  std::vector<uint8_t> f;
  char h[96];
  for (;;)
  {
    if (xSemaphoreTake(frame_mux, pdMS_TO_TICKS(100)))
    {
      f = latest_jpeg;
      xSemaphoreGive(frame_mux);
    }
    if (f.empty())
      continue;
    int n = snprintf(h, sizeof(h), "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", (unsigned)f.size());
    if (httpd_resp_send_chunk(r, h, n) != ESP_OK || httpd_resp_send_chunk(r, (char *)f.data(), f.size()) != ESP_OK || httpd_resp_send_chunk(r, "\r\n", 2) != ESP_OK)
      break;
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  return ESP_OK;
}
static esp_err_t control(httpd_req_t *r)
{
  char q[80], v[16];
  float x = 0, y = 0;
  if (httpd_req_get_url_query_str(r, q, sizeof(q)) == ESP_OK)
  {
    if (httpd_query_key_value(q, "x", v, sizeof(v)) == ESP_OK)
      x = strtof(v, nullptr);
    if (httpd_query_key_value(q, "y", v, sizeof(v)) == ESP_OK)
      y = strtof(v, nullptr);
  }
  if (xSemaphoreTake(control_mux, pdMS_TO_TICKS(20)))
  {
    automatic = false;
    yaw = std::clamp(yaw + x * 4, 1.f, 344.f);
    /* Screen Y grows downward, while the physical pitch-positive direction
     * raises the camera. Invert joystick Y for intuitive drag direction. */
    pitch = std::clamp(pitch - y * 3, 1.f, 175.f);
    send_target();
    xSemaphoreGive(control_mux);
  }
  ESP_LOGI(TAG, "HTTP CONTROL x=%.2f y=%.2f", (double)x, (double)y);
  return httpd_resp_sendstr(r, "OK UART queued");
}
static esp_err_t mode(httpd_req_t *r)
{
  char q[24], v[4];
  if (httpd_req_get_url_query_str(r, q, sizeof(q)) == ESP_OK && httpd_query_key_value(q, "auto", v, sizeof(v)) == ESP_OK)
    automatic = atoi(v) != 0;
  ESP_LOGI(TAG, "HTTP MODE auto=%d", automatic);
  return httpd_resp_sendstr(r, "OK");
}
static esp_err_t request_motion(httpd_req_t *r, uint8_t flags, const char *name)
{
  if (xSemaphoreTake(control_mux, pdMS_TO_TICKS(20)))
  {
    automatic = false;
    if (flags & 0x09U) {
      yaw = 180.0f;
      pitch = 90.0f;
    }
    send_target(flags);
    xSemaphoreGive(control_mux);
  }
  ESP_LOGI(TAG, "HTTP %s", name);
  return httpd_resp_sendstr(r, "OK UART queued");
}
static esp_err_t center(httpd_req_t *r) { return request_motion(r, 0x01U, "CENTER"); }
static esp_err_t calibrate(httpd_req_t *r) { return request_motion(r, 0x08U, "CAL"); }
static esp_err_t angles(httpd_req_t *r)
{
  char q[80], v[16];
  float new_yaw = yaw, new_pitch = pitch;
  if (httpd_req_get_url_query_str(r, q, sizeof(q)) == ESP_OK) {
    if (httpd_query_key_value(q, "yaw", v, sizeof(v)) == ESP_OK) new_yaw = strtof(v, nullptr);
    if (httpd_query_key_value(q, "pitch", v, sizeof(v)) == ESP_OK) new_pitch = strtof(v, nullptr);
  }
  if (new_yaw < 1.0f || new_yaw > 344.0f || new_pitch < 1.0f || new_pitch > 175.0f)
    return httpd_resp_send_err(r, HTTPD_400_BAD_REQUEST, "range");
  if (xSemaphoreTake(control_mux, pdMS_TO_TICKS(20))) {
    automatic = false;
    yaw = new_yaw;
    pitch = new_pitch;
    send_target();
    xSemaphoreGive(control_mux);
  }
  ESP_LOGI(TAG, "HTTP ANGLES yaw=%.1f pitch=%.1f", (double)new_yaw, (double)new_pitch);
  return httpd_resp_sendstr(r, "OK UART queued");
}
static void web_init()
{
  esp_netif_init();
  esp_event_loop_create_default();
  esp_netif_create_default_wifi_ap();
  wifi_init_config_t i = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&i);
  wifi_config_t c{};
  strcpy((char *)c.ap.ssid, "Gimbal-ReCamera");
  strcpy((char *)c.ap.password, "gimbal123");
  c.ap.ssid_len = 15;
  c.ap.channel = 6;
  c.ap.max_connection = 3;
  c.ap.authmode = WIFI_AUTH_WPA2_PSK;
  esp_wifi_set_mode(WIFI_MODE_AP);
  esp_wifi_set_config(WIFI_IF_AP, &c);
  esp_wifi_start();
  httpd_config_t control_config = HTTPD_DEFAULT_CONFIG();
  control_config.stack_size = 8192;
  control_config.max_uri_handlers = 8;
  httpd_handle_t control_server = nullptr;
  ESP_ERROR_CHECK(httpd_start(&control_server, &control_config));
  httpd_uri_t controls[] = {{"/", HTTP_GET, root, 0}, {"/status", HTTP_GET, status, 0}, {"/control", HTTP_GET, control, 0}, {"/angles", HTTP_GET, angles, 0}, {"/mode", HTTP_GET, mode, 0}, {"/center", HTTP_GET, center, 0}, {"/cal", HTTP_GET, calibrate, 0}};
  for (auto &uri : controls)
    ESP_ERROR_CHECK(httpd_register_uri_handler(control_server, &uri));

  /* MJPEG handlers do not return until the browser closes the connection.
   * Keep streaming on its own server task so refresh/reconnect can never
   * block /start, /control, or /status on port 80. */
  httpd_config_t stream_config = HTTPD_DEFAULT_CONFIG();
  stream_config.server_port = 81;
  stream_config.ctrl_port = 32769;
  stream_config.stack_size = 8192;
  stream_config.max_uri_handlers = 1;
  httpd_handle_t stream_server = nullptr;
  ESP_ERROR_CHECK(httpd_start(&stream_server, &stream_config));
  httpd_uri_t stream_uri = {"/stream", HTTP_GET, stream, 0};
  ESP_ERROR_CHECK(httpd_register_uri_handler(stream_server, &stream_uri));
  ESP_LOGI(TAG, "HTTP control=:80 stream=:81; http://192.168.4.1");
}
extern "C" void app_main()
{
  esp_err_t e = nvs_flash_init();
  if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    nvs_flash_erase();
    nvs_flash_init();
  }
  frame_mux = xSemaphoreCreateMutex();
  control_mux = xSemaphoreCreateMutex();
  uart_tx_queue = xQueueCreate(16, sizeof(QueuedTarget));
  ESP_ERROR_CHECK((frame_mux && control_mux && uart_tx_queue) ? ESP_OK : ESP_ERR_NO_MEM);
  uart_config_t u{.baud_rate = 115200, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT};
  ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 2048, 0, 0, nullptr, 0));
  ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &u));
  ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, GPIO_NUM_43, GPIO_NUM_44,
                               UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  ESP_LOGI(TAG, "BUILD uart-duplex-v3: UART1 D6/GPIO43 TX D7/GPIO44 RX 115200 8N1");
  ESP_ERROR_CHECK(camera_init());
  web_init();
  xTaskCreatePinnedToCore(uart_tx_task, "uart_tx", 3072, nullptr, 7, nullptr, 1);
  xTaskCreatePinnedToCore(uart_rx_task, "uart_rx", 4096, nullptr, 7, nullptr, 1);
  xTaskCreatePinnedToCore(capture, "capture", 4096, nullptr, 5, nullptr, 0);
  xTaskCreatePinnedToCore(infer, "face_detect", 12288, nullptr, 4, nullptr, 1);
}
