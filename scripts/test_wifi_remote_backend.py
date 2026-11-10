"""Run the real remote-control C++ with host doubles for ESP-IDF hardware APIs."""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parent.parent
STUB = r'''
#pragma once
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <functional>
#include <cassert>
#include <iostream>
using esp_err_t = int;
using httpd_handle_t = void*;
using BaseType_t = int;
constexpr int ESP_OK=0, ESP_FAIL=-1, HTTP_GET=0, HTTP_POST=1, pdPASS=1;
constexpr int HTTPD_WS_TYPE_TEXT=1, HTTPD_WS_TYPE_BINARY=2;
constexpr int WIFI_MODE_APSTA=3, WIFI_AUTH_WPA2_PSK=3, WIFI_IF_AP=1, SHUT_RDWR=2;
#define ESP_LOGI(...) ((void)0)
#define ESP_LOGW(...) ((void)0)
#define ESP_LOGE(...) ((void)0)
#define pdMS_TO_TICKS(x) (x)
struct httpd_req_t { httpd_handle_t handle; int method; int fd; };
struct httpd_ws_frame_t { bool final=false, fragmented=false; int type=0; uint8_t* payload=nullptr; size_t len=0; };
struct httpd_uri_t { const char* uri; int method; esp_err_t(*handler)(httpd_req_t*); void* user_ctx; bool is_websocket; };
struct httpd_config_t { int server_port=80, max_open_sockets=7, send_wait_timeout=5, recv_wait_timeout=5; bool lru_purge_enable=false; void(*close_fn)(httpd_handle_t,int)=nullptr; };
#define HTTPD_DEFAULT_CONFIG() httpd_config_t{}
struct wifi_ap_record_t {};
struct esp_netif_t {};
struct wifi_config_t { struct {uint8_t ssid[32], password[64]; size_t ssid_len; int channel, max_connection, authmode;} ap; };
static int64_t now_us=100;
static std::string inbound;
static int inbound_type=HTTPD_WS_TYPE_TEXT, send_result=ESP_OK, queue_result=ESP_OK;
static int task_result=pdPASS;
static std::vector<httpd_uri_t> routes;
static std::vector<std::string> commands, images;
static std::vector<int> closed, shut;
static std::vector<std::pair<void(*)(void*),void*>> work;
static std::function<void()> on_delay;
static httpd_config_t server_config;
int64_t esp_timer_get_time(){return now_us;}
int httpd_req_to_sockfd(httpd_req_t* r){return r->fd;}
int httpd_resp_set_type(httpd_req_t*,const char*){return ESP_OK;}
int httpd_resp_send(httpd_req_t*,const char*,size_t){return ESP_OK;}
int httpd_ws_recv_frame(httpd_req_t*,httpd_ws_frame_t* p,size_t max){
    if(!max){p->len=inbound.size();p->type=inbound_type;return ESP_OK;}
    if(max<inbound.size())return ESP_FAIL;
    memcpy(p->payload,inbound.data(),inbound.size());return ESP_OK;
}
int httpd_ws_send_frame_async(httpd_handle_t,int,httpd_ws_frame_t* p){
    assert(p->type==HTTPD_WS_TYPE_BINARY);
    images.emplace_back(reinterpret_cast<char*>(p->payload),p->len);return send_result;
}
int httpd_queue_work(httpd_handle_t,void(*fn)(void*),void* p){
    if(queue_result==ESP_OK)work.emplace_back(fn,p);return queue_result;
}
int httpd_start(httpd_handle_t* h,const httpd_config_t* c){*h=reinterpret_cast<void*>(1);server_config=*c;return ESP_OK;}
int httpd_stop(httpd_handle_t){return ESP_OK;}
int httpd_register_uri_handler(httpd_handle_t,const httpd_uri_t* r){routes.push_back(*r);return ESP_OK;}
int close(int fd){closed.push_back(fd);return 0;}
int shutdown(int fd,int){shut.push_back(fd);return 0;}
void vTaskDelay(int){if(on_delay)on_delay();}
void vTaskDelete(void*){}
int xTaskCreate(void(*)(void*),const char*,unsigned,void*,unsigned,void*){return task_result;}
int esp_wifi_sta_get_ap_info(wifi_ap_record_t*){return ESP_FAIL;}
esp_netif_t* esp_netif_get_handle_from_ifkey(const char*){return nullptr;}
esp_netif_t* esp_netif_create_default_wifi_ap(){return nullptr;}
int esp_wifi_set_mode(int){return ESP_OK;}
int esp_wifi_set_config(int,wifi_config_t*){return ESP_OK;}
const char* esp_err_to_name(int){return "error";}
// 统计取帧缓冲区：每一条路径都必须归还，否则就是内存泄漏。
static int jpeg_allocs=0, jpeg_frees=0;
static uint8_t* tracked_malloc(size_t n){++jpeg_allocs;return static_cast<uint8_t*>(malloc(n));}
#define free(p) (jpeg_frees += ((p) != nullptr), ::free(p))
'''
TEST = r'''
#include "SOURCE"
static httpd_req_t request{reinterpret_cast<void*>(1),HTTP_GET,7};
static int captures=0;
static void drain(){auto jobs=work;work.clear();for(auto job:jobs)job.first(job.second);}
static void connect_client(){request.method=HTTP_GET;assert(ws_handler(&request)==ESP_OK);request.method=HTTP_POST;}
static int receive(const std::string& text){inbound=text;return ws_handler(&request);}
static void run_worker(int ticks=3){
    int count=0;s_stream_running=true;s_stream_active=true;
    on_delay=[&](){if(++count>=ticks)s_stream_running=false;};
    stream_task(nullptr);on_delay=nullptr;
}
int main(){
    setvbuf(stdout,nullptr,_IONBF,0);
    s_send=[](const std::string& cmd){commands.push_back(cmd);};
    s_capture_jpeg=[](size_t* len){++captures;*len=4;auto* p=tracked_malloc(4);memcpy(p,"JPEG",4);return p;};
    start_httpd();
    assert(s_server && routes.size()==2 && routes[1].is_websocket && std::string(routes[1].uri)=="/ws");
    connect_client();
    assert(receive("x0.25y-0.50")==ESP_OK && commands.back()=="x0.25y-0.50");
    assert(receive("x0y0")==ESP_OK && commands.back()=="x0y0");
    auto count=commands.size();now_us=200;
    assert(receive("PING")==ESP_OK && commands.size()==count && s_last_seen==200);
    std::cout<<"PASS official coordinates, stop and heartbeat\n";
    for(auto bad:{"xnan y0","x2y0","x0y0bad","w8"})assert(receive(bad)==ESP_FAIL);
    inbound_type=HTTPD_WS_TYPE_BINARY;assert(receive("x0y1")==ESP_FAIL);inbound_type=HTTPD_WS_TYPE_TEXT;
    assert(receive(std::string(48,'x'))==ESP_FAIL && commands.size()==count);
    std::cout<<"PASS invalid control frames do not reach UART\n";
    httpd_req_t second{request.handle,HTTP_GET,8};assert(ws_handler(&second)==ESP_FAIL);
    server_config.close_fn(s_server,8);assert(s_client_fd==7 && commands.size()==count);
    server_config.close_fn(s_server,7);assert(s_client_fd==-1 && commands.back()=="x0y0" && closed.back()==7);
    std::cout<<"PASS one controller, correct disconnect stop\n";
    connect_client();run_worker();assert(captures==1 && work.size()==1 && s_frame_pending);
    s_stream_running=true;drain();assert(images.size()==1 && images.back()=="JPEG" && !s_frame_pending);
    std::cout<<"PASS camera worker queues at most one owned JPEG\n";
    run_worker();assert(work.size()==1);auto old_session=s_session.load();
    close_client(s_server,7);connect_client();assert(s_session!=old_session);
    s_stream_running=true;drain();assert(images.size()==1 && !s_frame_pending);
    std::cout<<"PASS old frame is discarded after same socket reconnects\n";
    queue_result=ESP_FAIL;run_worker(1);assert(work.empty() && !s_frame_pending);queue_result=ESP_OK;
    std::cout<<"PASS failed enqueue releases pending frame\n";
    run_worker(1);s_stream_running=true;send_result=ESP_FAIL;drain();
    assert(s_client_fd==-1 && shut.back()==7 && commands.back()=="x0y0");send_result=ESP_OK;
    std::cout<<"PASS send failure stops chassis and shuts socket\n";
    connect_client();s_capture_jpeg=[](size_t* len)->uint8_t*{*len=0;return nullptr;};
    run_worker(1);now_us+=6000000;s_stream_running=true;drain();
    assert(s_client_fd==-1 && commands.back()=="x0y0");
    std::cout<<"PASS heartbeat expiry also works without camera frames\n";
    s_stream_active=false;WifiRemoteControl::Stop();assert(!s_server && !s_frame_pending);
    assert(jpeg_allocs>0 && jpeg_allocs==jpeg_frees);
    std::cout<<"PASS every captured frame buffer is released (alloc="<<jpeg_allocs<<")\n";
    task_result=ESP_FAIL;start_httpd();assert(!s_server && !s_stream_running && !s_stream_active);
    std::cout<<"PASS worker startup failure cleans up server\n";
}
'''

with tempfile.TemporaryDirectory(prefix='sparkbot-ws-') as folder:
    tmp = Path(folder)
    (tmp/'esp_test.h').write_text(STUB, encoding='utf-8')
    for name in ['esp_log.h','esp_wifi.h','esp_netif.h','esp_wifi_default.h',
                 'esp_http_server.h','esp_timer.h','lwip/sockets.h',
                 'freertos/FreeRTOS.h','freertos/task.h']:
        target=tmp/name
        target.parent.mkdir(parents=True,exist_ok=True)
        target.write_text('#include "esp_test.h"\n',encoding='utf-8')
    (tmp/'test.cpp').write_text(TEST.replace('SOURCE',(ROOT/'components/wifi_remote_control/wifi_remote_control.cc').as_posix()),encoding='utf-8')
    gcc=shutil.which('g++') or shutil.which('clang++')
    if gcc:
        exe=tmp/('test.exe' if os.name=='nt' else 'test')
        subprocess.run([gcc,'-std=c++17','-Wall','-I',str(tmp),'test.cpp','-o',str(exe)],cwd=tmp,check=True)
        subprocess.run([str(exe)],cwd=tmp,check=True)
    elif os.name=='nt':
        # 没有 g++/clang++ 时才回退 MSVC；MSVC 链接依赖 Windows SDK，缺 SDK 会直接失败。
        installs=list(Path('C:/Program Files (x86)/Microsoft Visual Studio/2022').glob('*/VC/Auxiliary/Build/vcvars64.bat'))
        if not installs:
            raise SystemExit('no C++17 compiler: install g++ on PATH (MSVC Build Tools not found either)')
        (tmp/'build.cmd').write_text(f'@echo off\ncall "{installs[0]}" >nul\ncl /nologo /std:c++17 /EHsc /utf-8 /I"{tmp}" test.cpp /Fe:test.exe\n',encoding='utf-8')
        subprocess.run(['cmd','/c',str(tmp/'build.cmd')],cwd=tmp,check=True)
        subprocess.run([str(tmp/'test.exe')],cwd=tmp,check=True)
    else:
        raise SystemExit('C++17 compiler not found')
