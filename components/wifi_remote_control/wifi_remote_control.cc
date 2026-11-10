#include "wifi_remote_control.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <atomic>
#include <cmath>
#include <new>
#include <esp_timer.h>
#include <lwip/sockets.h>

#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <esp_wifi_default.h>
#include <esp_http_server.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "wifi_remote";

static httpd_handle_t s_server = nullptr;
static std::atomic<bool> s_stream_running{false};
static std::atomic<bool> s_stream_active{false};
static std::atomic<bool> s_frame_pending{false};
static std::atomic<uint32_t> s_session{0};
static uint32_t s_next_session = 0;
static int s_client_fd = -1;
static int64_t s_last_seen = 0;

struct VideoFrame {
    uint32_t session;
    uint8_t* data;
    size_t len;
};
static std::function<void(const std::string&)> s_send;
static WifiRemoteControl::CaptureJpegFn s_capture_jpeg;

// ---------------------------------------------------------------------------
// 手机遥控页面（全屏摇杆，适配触屏）
// ---------------------------------------------------------------------------
static const char kHtml[] = R"HTML(<!doctype html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>SparkBot 遥控</title>
<style>
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{margin:0;min-height:100vh;background:#0d1117;color:#e6edf3;
     font-family:system-ui,-apple-system,sans-serif;overflow:hidden;
     display:flex;flex-direction:column;align-items:center;justify-content:center;
     gap:10px;padding:10px;touch-action:none;user-select:none}
h1{font-size:14px;font-weight:500;color:#8b949e;margin:0;letter-spacing:2px}
#view{width:min(76vw,240px);aspect-ratio:1/1;background:#000;border-radius:12px;
      overflow:hidden;border:1px solid #30363d}
#view img{width:100%;height:100%;object-fit:cover;display:block}
#pad{position:relative;width:min(58vw,180px);aspect-ratio:1/1;border-radius:50%;
     background:radial-gradient(circle at 50% 40%,#161b22,#0d1117 70%);
     border:1.5px solid #30363d;touch-action:none}
#knob{position:absolute;left:50%;top:50%;width:70px;height:70px;margin:-35px 0 0 -35px;
      border-radius:50%;background:linear-gradient(145deg,#1f6feb,#0b3d91);
      border:1.5px solid #58a6ff;box-shadow:0 0 20px rgba(56,139,253,.5);
      transition:transform .04s linear}
#val{font-size:12px;color:#8b949e;font-variant-numeric:tabular-nums}
#hint{font-size:11px;color:#484f58}
#flips{display:flex;gap:8px}
#flips button{background:#21262d;color:#8b949e;border:1px solid #30363d;
              border-radius:6px;padding:5px 10px;font-size:11px;
              -webkit-tap-highlight-color:transparent}
#flips button.on{color:#58a6ff;border-color:#1f6feb}
</style></head><body>
<h1>SPARKBOT 遥控</h1>
<div id="view"><img id="cam" alt="camera"></div>
<div id="flips">
  <button id="fh" onclick="toggleH()">左右翻转</button>
  <button id="fv" onclick="toggleV()">上下翻转</button>
</div>
<div id="pad"><div id="knob"></div></div>
<div id="val">x 0.00 &nbsp; y 0.00</div>
<div id="hint">拖动摇杆控制 · 松手停止</div>
<script>
var pad=document.getElementById('pad'),knob=document.getElementById('knob'),
    val=document.getElementById('val'),hint=document.getElementById('hint'),
    dragging=false,last=0,currentX=0,currentY=0,pointerId=null;
function stickR(){return pad.clientWidth/2-8;}
var cam=document.getElementById('cam'),fhOn=true,fvOn=false;
function applyFlip(){
  cam.style.transform='scaleX('+(fhOn?-1:1)+') scaleY('+(fvOn?-1:1)+')';
  document.getElementById('fh').className=fhOn?'on':'';
  document.getElementById('fv').className=fvOn?'on':'';
}
function toggleH(){fhOn=!fhOn;applyFlip();}
function toggleV(){fvOn=!fvOn;applyFlip();}
applyFlip();
var ws=null,imageUrl=null,imageBusy=false;
cam.onload=cam.onerror=function(){imageBusy=false;};
function send(x,y){
  if(ws&&ws.readyState===WebSocket.OPEN&&ws.bufferedAmount===0)
    ws.send('x'+x.toFixed(2)+'y'+y.toFixed(2));
}
function sendStop(){
  if(ws&&ws.readyState===WebSocket.OPEN)ws.send('x0y0');
}
function resetStick(){
  dragging=false;pointerId=null;currentX=0;currentY=0;
  knob.style.transform='translate(0,0)';val.textContent='x 0.00   y 0.00';
}
function release(){resetStick();sendStop();}
function connect(){
  ws=new WebSocket((location.protocol==='https:'?'wss://':'ws://')+location.host+'/ws');
  ws.onopen=function(){resetStick();sendStop();hint.textContent='已连接 · 拖动摇杆控制 · 松手停止';};
  ws.onmessage=function(event){
    if(typeof event.data==='string'||imageBusy)return;
    var previous=imageUrl;
    imageUrl=URL.createObjectURL(new Blob([event.data],{type:'image/jpeg'}));
    imageBusy=true;cam.src=imageUrl;
    if(previous)URL.revokeObjectURL(previous);
  };
  ws.onerror=function(){ws.close();};
  ws.onclose=function(){
    resetStick();imageBusy=false;
    if(imageUrl){URL.revokeObjectURL(imageUrl);imageUrl=null;}
    hint.textContent='连接断开，正在重连…';setTimeout(connect,1000);
  };
}
connect();
setInterval(function(){if(dragging)send(currentX,currentY);},150);
setInterval(function(){
  if(ws&&ws.readyState===WebSocket.OPEN&&ws.bufferedAmount===0)ws.send('PING');
},1000);
function handle(cx,cy){
  var r=pad.getBoundingClientRect(),R=stickR();
  var dx=cx-(r.left+r.width/2),dy=cy-(r.top+r.height/2);
  var d=Math.sqrt(dx*dx+dy*dy);
  if(d>R){dx=dx/d*R;dy=dy/d*R;}
  knob.style.transform='translate('+dx+'px,'+dy+'px)';
  currentX=dx/R;currentY=-dy/R;
  val.textContent='x '+currentX.toFixed(2)+'   y '+currentY.toFixed(2);
  var now=performance.now();
  if(now-last>150){last=now;send(currentX,currentY);}
}
pad.addEventListener('pointerdown',function(e){
  if(dragging||!ws||ws.readyState!==WebSocket.OPEN)return;
  dragging=true;pointerId=e.pointerId;
  if(pad.setPointerCapture)pad.setPointerCapture(e.pointerId);
  handle(e.clientX,e.clientY);
});
pad.addEventListener('pointermove',function(e){
  if(dragging&&e.pointerId===pointerId)handle(e.clientX,e.clientY);
});
['pointerup','pointercancel','lostpointercapture'].forEach(function(name){
  pad.addEventListener(name,function(e){if(e.pointerId===pointerId)release();});
});
window.addEventListener('blur',release);
window.addEventListener('pagehide',release);
document.addEventListener('visibilitychange',function(){if(document.hidden)release();});
</script></body></html>)HTML";

// ---------------------------------------------------------------------------
// HTTP 处理
// ---------------------------------------------------------------------------
static esp_err_t root_handler(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, kHtml, strlen(kHtml));
    return ESP_OK;
}

static void disconnect_client() {
    s_session.store(0);
    s_client_fd = -1;
    if (s_send) s_send("x0y0");
}

static void close_client(httpd_handle_t server, int fd) {
    if (fd == s_client_fd) disconnect_client();
    close(fd);
}

static esp_err_t ws_handler(httpd_req_t* req) {
    int fd = httpd_req_to_sockfd(req);
    if (req->method == HTTP_GET) {
        if (s_client_fd != -1) return ESP_FAIL;
        s_client_fd = fd;
        s_last_seen = esp_timer_get_time();
        if (++s_next_session == 0) ++s_next_session;
        s_session.store(s_next_session);
        ESP_LOGI(TAG, "WebSocket control connected");
        return ESP_OK;
    }
    if (fd != s_client_fd) return ESP_FAIL;
    httpd_ws_frame_t packet = {};
    esp_err_t err = httpd_ws_recv_frame(req, &packet, 0);
    if (err != ESP_OK) return err;
    if (packet.type != HTTPD_WS_TYPE_TEXT || packet.len == 0 || packet.len >= 48)
        return ESP_FAIL;
    char text[48] = {};
    packet.payload = reinterpret_cast<uint8_t*>(text);
    err = httpd_ws_recv_frame(req, &packet, sizeof(text) - 1);
    if (err != ESP_OK) return err;
    if (strcmp(text, "PING") == 0) {
        s_last_seen = esp_timer_get_time();
        return ESP_OK;
    }
    float x = 0, y = 0;
    char trailing = 0;
    if (sscanf(text, "x%f y%f %c", &x, &y, &trailing) != 2 ||
        !std::isfinite(x) || !std::isfinite(y) || fabsf(x) > 1 || fabsf(y) > 1)
        return ESP_FAIL;
    s_last_seen = esp_timer_get_time();
    if (s_send) s_send(text);
    return ESP_OK;
}

// HTTP 任务只发送已编码的单帧；取帧和 JPEG 编码在独立任务中完成。
static void send_video_frame(void* arg) {
    auto* frame = static_cast<VideoFrame*>(arg);
    if (s_stream_running.load() && frame->session == s_session.load() && s_client_fd >= 0) {
        esp_err_t err = ESP_OK;
        if (esp_timer_get_time() - s_last_seen > 5000000) {
            err = ESP_FAIL;
        } else if (frame->data != nullptr && frame->len > 0) {
            httpd_ws_frame_t packet = {};
            packet.type = HTTPD_WS_TYPE_BINARY;
            packet.payload = frame->data;
            packet.len = frame->len;
            err = httpd_ws_send_frame_async(s_server, s_client_fd, &packet);
        }
        if (err != ESP_OK) {
            int fd = s_client_fd;
            disconnect_client();
            shutdown(fd, SHUT_RDWR);
        }
    }
    free(frame->data);
    delete frame;
    s_frame_pending.store(false);
}

static void stream_task(void*) {
    while (s_stream_running.load()) {
        uint32_t session = s_session.load();
        if (session != 0 && !s_frame_pending.load()) {
            auto* frame = new (std::nothrow) VideoFrame{session, nullptr, 0};
            if (frame != nullptr) {
                if (s_capture_jpeg) frame->data = s_capture_jpeg(&frame->len);
                s_frame_pending.store(true);
                if (!s_stream_running.load() || httpd_queue_work(s_server, send_video_frame, frame) != ESP_OK) {
                    free(frame->data);
                    delete frame;
                    s_frame_pending.store(false);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(150));
    }
    s_stream_active.store(false);
    vTaskDelete(nullptr);
}

static void start_httpd() {
    if (s_server != nullptr) return;
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    cfg.max_open_sockets = 3;
    cfg.lru_purge_enable = true;
    cfg.send_wait_timeout = 1;
    cfg.recv_wait_timeout = 1;
    cfg.close_fn = close_client;
    if (httpd_start(&s_server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed");
        s_server = nullptr;
        return;
    }
    httpd_uri_t root = {};
    root.uri = "/";
    root.method = HTTP_GET;
    root.handler = root_handler;
    httpd_uri_t ws = {};
    ws.uri = "/ws";
    ws.method = HTTP_GET;
    ws.handler = ws_handler;
    ws.is_websocket = true;
    if (httpd_register_uri_handler(s_server, &root) != ESP_OK ||
        httpd_register_uri_handler(s_server, &ws) != ESP_OK) {
        httpd_stop(s_server);
        s_server = nullptr;
        ESP_LOGE(TAG, "register remote handlers failed");
        return;
    }
    s_stream_running.store(true);
    s_stream_active.store(true);
    // 栈要够 frame2jpg 编码用：4K 偏紧，编码偶发栈溢出会直接把设备打重启
    if (xTaskCreate(stream_task, "ws_camera", 8192, nullptr, 3, nullptr) != pdPASS) {
        s_stream_active.store(false);
        s_stream_running.store(false);
        httpd_stop(s_server);
        s_server = nullptr;
        ESP_LOGE(TAG, "create camera task failed");
        return;
    }
    ESP_LOGI(TAG, "WebSocket remote ready on port 80 /ws");
}

// ---------------------------------------------------------------------------
// 开热点（延迟启动，避免和 STA 抢信道）
// ---------------------------------------------------------------------------
static void ap_task(void* arg) {
    vTaskDelay(pdMS_TO_TICKS(6000));  // 等 WiFi 状态稳定

    // 1) 只在 STA 已连接时启用。设备处于配网模式时，WifiConfigurationAp 已经创建了
    //    WIFI_AP_DEF 和 80 端口服务器，这里再创建会触发 assert（duplicate key）→ 重启。
    wifi_ap_record_t sta_info;
    if (esp_wifi_sta_get_ap_info(&sta_info) != ESP_OK) {
        ESP_LOGW(TAG, "STA 未连接（可能处于配网模式），跳过遥控热点");
        vTaskDelete(nullptr);
        return;
    }

    // 2) 复用已有 AP netif，不要重复创建
    esp_netif_t* ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (ap_netif == nullptr) {
        ap_netif = esp_netif_create_default_wifi_ap();
    }
    if (ap_netif == nullptr) {
        ESP_LOGE(TAG, "无法获取 AP netif，放弃遥控热点");
        vTaskDelete(nullptr);
        return;
    }

    // 3) 切 APSTA，保留原有的 STA 连接
    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set WIFI_MODE_APSTA failed: %s", esp_err_to_name(err));
        vTaskDelete(nullptr);
        return;
    }

    wifi_config_t ap_cfg = {};
    const char* ssid = "SparkBot-Car";
    const char* pass = "12345678";
    strncpy(reinterpret_cast<char*>(ap_cfg.ap.ssid), ssid, sizeof(ap_cfg.ap.ssid));
    strncpy(reinterpret_cast<char*>(ap_cfg.ap.password), pass, sizeof(ap_cfg.ap.password));
    ap_cfg.ap.ssid_len = strlen(ssid);
    ap_cfg.ap.channel = 0;  // 0 = 跟随 STA 当前信道
    ap_cfg.ap.max_connection = 4;
    ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;

    err = esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set AP config failed: %s", esp_err_to_name(err));
        vTaskDelete(nullptr);
        return;
    }
    ESP_LOGI(TAG, "AP configured: ssid=%s pass=%s", ssid, pass);

    start_httpd();
    ESP_LOGI(TAG, "phone -> wifi '%s' / '%s' -> http://192.168.4.1", ssid, pass);
    vTaskDelete(nullptr);
}

bool WifiRemoteControl::Start(std::function<void(const std::string&)> send_cmd,
                              CaptureJpegFn capture_jpeg) {
    s_send = std::move(send_cmd);
    s_capture_jpeg = std::move(capture_jpeg);
    BaseType_t ok = xTaskCreate(ap_task, "wifi_remote", 4096, nullptr, 3, nullptr);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "create ap_task failed");
        return false;
    }
    return true;
}

void WifiRemoteControl::Stop() {
    s_stream_running.store(false);
    while (s_stream_active.load() || s_frame_pending.load()) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (s_server != nullptr) {
        httpd_stop(s_server);
        s_server = nullptr;
    }
}
