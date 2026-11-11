#ifndef WIFI_REMOTE_CONTROL_H
#define WIFI_REMOTE_CONTROL_H

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

/**
 * 手机网页遥控
 *
 * 工作方式：
 *   1. 设备开一个 WiFi 热点（默认 SSID: SparkBot-Car / 密码: 12345678）
 *   2. 手机连上这个热点，浏览器打开 http://192.168.4.1
 *   3. 页面上方是摄像头实时画面，下方摇杆，拖动即遥控
 *
 * 注意：使用 APSTA 模式，热点与原有 WiFi 连接共存（可能影响稳定性，需实测）
 */
class WifiRemoteControl {
public:
    // 抓一帧并转成 JPEG，返回 malloc 缓冲区（调用方 free），失败返回 nullptr
    using CaptureJpegFn = std::function<uint8_t*(size_t*)>;
    // 读电池电量与充电状态，返回 false 表示当前读不到
    using BatteryFn = std::function<bool(int&, bool&)>;

    /**
     * @param send_cmd      收到摇杆位置后如何下发指令，例如 "x0.5 y1.0"
     * @param capture_jpeg  可选：摄像头取帧回调，提供后网页可显示实时画面
     * @param read_battery  可选：电池读取回调，提供后网页顶部显示电量
     */
    static bool Start(std::function<void(const std::string&)> send_cmd,
                      CaptureJpegFn capture_jpeg = nullptr,
                      BatteryFn read_battery = nullptr);

    static void Stop();
};

#endif  // WIFI_REMOTE_CONTROL_H
