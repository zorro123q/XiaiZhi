#include "wifi_remote_control.h"

#include <cstdio>
#include <cstring>

#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_netif.h>
#include <esp_wifi_default.h>
#include <esp_http_server.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "wifi_remote";

static httpd_handle_t s_server = nullptr;
static std::function<void(const std::string&)> s_send;

// ---------------------------------------------------------------------------
// 手机遥控页面（全屏摇杆，适配触屏）
// ---------------------------------------------------------------------------
static const char kHtml[] = R"HTML(<!doctype html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>SparkBot 遥控</title>
<style>
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{margin:0;height:100vh;background:#0d1117;color:#e6edf3;
     font-family:system-ui,-apple-system,sans-serif;overflow:hidden;
     display:flex;flex-direction:column;align-items:center;justify-content:center;
     touch-action:none;user-select:none}
h1{font-size:15px;font-weight:500;color:#8b949e;margin:0 0 20px;letter-spacing:2px}
#pad{position:relative;width:250px;height:250px;border-radius:50%;
     background:radial-gradient(circle at 50% 40%,#161b22,#0d1117 70%);
     border:1.5px solid #30363d;touch-action:none}
#knob{position:absolute;left:50%;top:50%;width:92px;height:92px;margin:-46px 0 0 -46px;
      border-radius:50%;background:linear-gradient(145deg,#1f6feb,#0b3d91);
      border:1.5px solid #58a6ff;box-shadow:0 0 24px rgba(56,139,253,.5);
      transition:transform .04s linear}
#val{margin-top:18px;font-size:13px;color:#8b949e;font-variant-numeric:tabular-nums}
#hint{margin-top:8px;font-size:11px;color:#484f58}
</style></head><body>
<h1>SPARKBOT 遥控</h1>
<div id="pad"><div id="knob"></div></div>
<div id="val">x 0.00 &nbsp; y 0.00</div>
<div id="hint">按住摇杆拖动 · 松手停止</div>
<script>
var pad=document.getElementById('pad'),knob=document.getElementById('knob'),
    val=document.getElementById('val'),R=100,dragging=false,last=0;
function send(x,y){fetch('/ctrl?x='+x.toFixed(2)+'&y='+y.toFixed(2)).catch(function(){});}
function sendStop(){fetch('/stop').catch(function(){});}
function handle(cx,cy){
  var r=pad.getBoundingClientRect();
  var dx=cx-(r.left+r.width/2),dy=cy-(r.top+r.height/2);
  var d=Math.sqrt(dx*dx+dy*dy);
  if(d>R){dx=dx/d*R;dy=dy/d*R;}
  knob.style.transform='translate('+dx+'px,'+dy+'px)';
  var x=dx/R,y=-dy/R;
  val.textContent='x '+x.toFixed(2)+'   y '+y.toFixed(2);
  var now=performance.now();
  if(now-last>100){last=now;send(x,y);}
}
pad.addEventListener('pointerdown',function(e){dragging=true;
  if(pad.setPointerCapture)pad.setPointerCapture(e.pointerId);
  handle(e.clientX,e.clientY);});
pad.addEventListener('pointermove',function(e){if(dragging)handle(e.clientX,e.clientY);});
pad.addEventListener('pointerup',function(){dragging=false;
  knob.style.transform='translate(0,0)';val.textContent='x 0.00   y 0.00';sendStop();});
pad.addEventListener('pointercancel',function(){dragging=false;
  knob.style.transform='translate(0,0)';sendStop();});
</script></body></html>)HTML";

// ---------------------------------------------------------------------------
// HTTP 处理
// ---------------------------------------------------------------------------
static esp_err_t root_handler(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, kHtml, strlen(kHtml));
    return ESP_OK;
}

static esp_err_t ctrl_handler(httpd_req_t* req) {
    char query[80] = {0}, xbuf[16] = {0}, ybuf[16] = {0};
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "x", xbuf, sizeof(xbuf)) == ESP_OK &&
        httpd_query_key_value(query, "y", ybuf, sizeof(ybuf)) == ESP_OK) {
        char cmd[40];
        snprintf(cmd, sizeof(cmd), "x%s y%s", xbuf, ybuf);
        if (s_send) {
            s_send(cmd);
        }
    }
    httpd_resp_send(req, "ok", 2);
    return ESP_OK;
}

static esp_err_t stop_handler(httpd_req_t* req) {
    if (s_send) {
        s_send("x0.0 y0.0");
    }
    httpd_resp_send(req, "stopped", 7);
    return ESP_OK;
}

static void start_httpd() {
    if (s_server != nullptr) {
        return;
    }
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    cfg.max_open_sockets = 4;
    cfg.lru_purge_enable = true;

    if (httpd_start(&s_server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed");
        s_server = nullptr;
        return;
    }

    httpd_uri_t u_root = {};
    u_root.uri = "/";
    u_root.method = HTTP_GET;
    u_root.handler = root_handler;
    u_root.user_ctx = nullptr;
    httpd_register_uri_handler(s_server, &u_root);

    httpd_uri_t u_ctrl = {};
    u_ctrl.uri = "/ctrl";
    u_ctrl.method = HTTP_GET;
    u_ctrl.handler = ctrl_handler;
    u_ctrl.user_ctx = nullptr;
    httpd_register_uri_handler(s_server, &u_ctrl);

    httpd_uri_t u_stop = {};
    u_stop.uri = "/stop";
    u_stop.method = HTTP_GET;
    u_stop.handler = stop_handler;
    u_stop.user_ctx = nullptr;
    httpd_register_uri_handler(s_server, &u_stop);

    ESP_LOGI(TAG, "httpd started on port 80");
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

bool WifiRemoteControl::Start(std::function<void(const std::string&)> send_cmd) {
    s_send = std::move(send_cmd);
    BaseType_t ok = xTaskCreate(ap_task, "wifi_remote", 4096, nullptr, 3, nullptr);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "create ap_task failed");
        return false;
    }
    return true;
}

void WifiRemoteControl::Stop() {
    if (s_server != nullptr) {
        httpd_stop(s_server);
        s_server = nullptr;
    }
}
