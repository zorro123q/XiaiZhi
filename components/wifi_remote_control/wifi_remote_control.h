#ifndef WIFI_REMOTE_CONTROL_H
#define WIFI_REMOTE_CONTROL_H

#include <functional>
#include <string>

/**
 * 手机网页遥控
 *
 * 工作方式：
 *   1. 设备开一个 WiFi 热点（默认 SSID: SparkBot-Car / 密码: 12345678）
 *   2. 手机连上这个热点，浏览器打开 http://192.168.4.1
 *   3. 页面上拖动摇杆，设备把 "x<x> y<y>" 通过串口发给底盘
 *
 * 注意：使用 APSTA 模式，热点与原有 WiFi 连接共存（可能影响稳定性，需实测）
 */
class WifiRemoteControl {
public:
    /**
     * @param send_cmd 收到摇杆位置后如何下发指令，例如传入 "x0.5 y1.0"
     */
    static bool Start(std::function<void(const std::string&)> send_cmd);

    static void Stop();
};

#endif  // WIFI_REMOTE_CONTROL_H
