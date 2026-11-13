#include "wifi_remote_control.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cerrno>
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
static WifiRemoteControl::BatteryFn s_read_battery;
static int64_t s_last_battery_send = 0;

// 遥控热点与 HTTP 服务
//
// 端口选 8080 而不是 80，是因为配网组件 WifiConfigurationAp 固定占用 80 端口
// （http://192.168.4.1）。两边都在 WIFI_AP_DEF 上，如果撞端口，配网页会
// httpd_start 失败——那就把用户彻底锁在配网外面了。分开端口后两者可以共存：
// 配网页走 80，遥控页走 8080，谁都不影响谁。
static constexpr const char* kApSsid = "SparkBot-Car";
static constexpr const char* kApPass = "12345678";
static constexpr int kHttpPort = 8080;

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
#bat{font-size:12px;color:#3fb950;letter-spacing:0;margin-left:6px}
#bat.low{color:#f85149}
#bat.hide{display:none}
</style></head><body>
<h1>SPARKBOT 遥控 <span id="bat" class="hide"></span></h1>
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
var cam=document.getElementById('cam'),fhOn=true,fvOn=false,bat=document.getElementById('bat');
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
    if(typeof event.data==='string'){
      try{
        var msg=JSON.parse(event.data);
        if(typeof msg.level==='number'){
          bat.textContent=msg.charging?(msg.level+'% 充电中'):(msg.level+'%');
          bat.className=msg.level<20?'low':'';
        }
      }catch(e){}
      return;
    }
    if(imageBusy)return;
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

// 复用控制用的 WebSocket 下发一条文本帧：{"level":78,"charging":false}
// 走同一条连接，不额外占用 socket
static void send_battery_frame(void*) {
    if (s_server == nullptr || s_client_fd < 0 || !s_read_battery) return;
    int level = 0;
    bool charging = false;
    if (!s_read_battery(level, charging)) return;
    char text[64];
    int len = snprintf(text, sizeof(text), "{\"level\":%d,\"charging\":%s}",
                       level, charging ? "true" : "false");
    if (len <= 0) return;
    httpd_ws_frame_t packet = {};
    packet.type = HTTPD_WS_TYPE_TEXT;
    packet.payload = reinterpret_cast<uint8_t*>(text);
    packet.len = len;
    httpd_ws_send_frame_async(s_server, s_client_fd, &packet);
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
        // 借心跳顺带下发电量，10 秒一次
        if (s_read_battery && esp_timer_get_time() - s_last_battery_send > 10000000) {
            s_last_battery_send = esp_timer_get_time();
            httpd_queue_work(s_server, send_battery_frame, nullptr);
        }
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
    cfg.server_port = kHttpPort;
    // 默认 ctrl_port 是 32768（UDP）。实测在纯 STA 启动早期绑这个端口会报
    // EADDRINUSE(112)，httpd 直接起不来——虽然 server socket(8080/TCP) 已绑成功。
    // 显式指定成 8081，跟 server_port 挨着，避免撞到 lwip 的临时端口池。
    cfg.ctrl_port = kHttpPort + 1;
    // 一个页面 + 一条 WebSocket 视频流就占两个；留出余量给第二台手机或临时重连
    cfg.max_open_sockets = 5;
    cfg.lru_purge_enable = true;
    cfg.send_wait_timeout = 1;
    cfg.recv_wait_timeout = 1;
    cfg.close_fn = close_client;
    if (httpd_start(&s_server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed (port %d/ctrl %d): %s", kHttpPort, cfg.ctrl_port, strerror(errno));
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
    // 看守任务可能会重复拉起 httpd，取帧任务只能有一份，否则两个任务
    // 同时抓帧会双倍占 CPU/PSRAM
    if (!s_stream_running.load()) {
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
    }
    ESP_LOGI(TAG, "WebSocket remote ready on port %d /ws", kHttpPort);
}

// ---------------------------------------------------------------------------
// 遥控热点（常驻看守）
//
// 旧实现的两个毛病：
//   1) 固定等 6 秒后**一次性**检查 STA，没连上就直接放弃、永不重试。
//      开机连接慢于 6 秒（扫描 + 关联 + DHCP）时，热点就整个消失了。
//   2) 判据是"STA 已连接"，等于"没网时反而不给热点"——而没网（没有路由器、
//      在外面、路由器改了密码）恰恰是最需要手机直连遥控的时候。
//
// 现在改成常驻看守：只要 AP 没被配网组件占用，就保证 SparkBot-Car 存在；
// 它与是否连上路由器无关。AP 被改名成 Xiaozhi-XXXX 时不抢（那是配网页的
// 热点），但 HTTP 服务在 8080 上继续可用。
// ---------------------------------------------------------------------------

static bool sta_connected() {
    wifi_ap_record_t info;
    return esp_wifi_sta_get_ap_info(&info) == ESP_OK;
}

// 本次运行里 AP netif 是不是我们自己建的。
// 不能用 esp_wifi_get_config(WIFI_IF_AP) 的 SSID 直接判断"配网有没有在跑"：
// WiFi 驱动会把 AP 配置存进 NVS，重启后哪怕配网早就结束了，读出来仍是
// 上次的 "Xiaozhi-XXXX" —— 据此判断会把我们自己挡在门外（实测踩过）。
// AP netif 是运行期对象，不会跨重启存活，用它判断才准。
static bool s_ap_netif_ours = false;

static bool ap_owned_by_others() {
    if (!s_ap_netif_ours) {
        // 我们还没建过 AP netif：它却已经存在 → 配网组件建的，让给它
        return esp_netif_get_handle_from_ifkey("WIFI_AP_DEF") != nullptr;
    }
    // 我们的 netif 还在，但 SSID 被改成了别的名字 → 配网接管了
    wifi_config_t cfg = {};
    if (esp_wifi_get_config(WIFI_IF_AP, &cfg) != ESP_OK) {
        return false;
    }
    const char* current = reinterpret_cast<const char*>(cfg.ap.ssid);
    return current[0] != '\0' && strcmp(current, kApSsid) != 0;
}

static bool bring_up_remote_ap() {
    // 复用已有 AP netif，不要重复创建（配网组件可能已经建过一个）
    esp_netif_t* ap_netif = esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");
    if (ap_netif == nullptr) {
        ap_netif = esp_netif_create_default_wifi_ap();
    }
    if (ap_netif == nullptr) {
        ESP_LOGE(TAG, "无法获取 AP netif，放弃遥控热点");
        return false;
    }
    s_ap_netif_ours = true;

    // APSTA：保留 STA 那边已有的连接或重连流程，同时把 AP 挂上
    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set WIFI_MODE_APSTA failed: %s", esp_err_to_name(err));
        return false;
    }

    wifi_config_t ap_cfg = {};
    strncpy(reinterpret_cast<char*>(ap_cfg.ap.ssid), kApSsid, sizeof(ap_cfg.ap.ssid));
    strncpy(reinterpret_cast<char*>(ap_cfg.ap.password), kApPass, sizeof(ap_cfg.ap.password));
    ap_cfg.ap.ssid_len = strlen(kApSsid);
    // 连上路由器时跟随 STA 信道，手机少切一次；没连时信道未知，固定到 6
    ap_cfg.ap.channel = sta_connected() ? 0 : 6;
    ap_cfg.ap.max_connection = 4;
    ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;

    err = esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set AP config failed: %s", esp_err_to_name(err));
        return false;
    }
    ESP_LOGI(TAG, "AP configured: ssid=%s pass=%s", kApSsid, kApPass);

    start_httpd();
    if (s_server == nullptr) {
        return false;
    }
    ESP_LOGI(TAG, "phone -> wifi '%s' / '%s' -> http://192.168.4.1:%d (STA %s)",
             kApSsid, kApPass, kHttpPort, sta_connected() ? "connected" : "disconnected");
    return true;
}

static void remote_watch_task(void*) {
    vTaskDelay(pdMS_TO_TICKS(6000));  // 先让 WiFi 状态机跑起来，别和 STA 抢

    while (true) {
        if (ap_owned_by_others()) {
            // 配网模式：热点名归配网组件，不去覆盖；只把 8080 上的遥控服务拉起来
            if (s_server == nullptr) {
                start_httpd();
            }
        } else if (s_server == nullptr) {
            bring_up_remote_ap();
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

bool WifiRemoteControl::Start(std::function<void(const std::string&)> send_cmd,
                              CaptureJpegFn capture_jpeg,
                              BatteryFn read_battery) {
    s_send = std::move(send_cmd);
    s_capture_jpeg = std::move(capture_jpeg);
    s_read_battery = std::move(read_battery);
    BaseType_t ok = xTaskCreate(remote_watch_task, "wifi_remote", 4096, nullptr, 3, nullptr);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "create remote watch task failed");
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
