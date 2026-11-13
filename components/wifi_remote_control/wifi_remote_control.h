#ifndef WIFI_REMOTE_CONTROL_H
#define WIFI_REMOTE_CONTROL_H

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>

/**
 * 手机网页遥控
 *
 * 工作方式（两条路，任选其一）：
 *   1. 直连热点：设备开热点（默认 SSID: SparkBot-Car / 密码: 12345678），
 *      手机连上后浏览器打开 http://192.168.4.1:8080
 *   2. 局域网直连：手机和车连同一个路由器，打开 http://<车的IP>:8080
 *      （车的 IP 会显示在开机画面上，也可以直接问语音助手）
 *
 * 端口用 8080 而不是 80：80 被配网组件 WifiConfigurationAp 占着，
 * 一旦撞端口，配网页会启动失败 —— 设备就再也配不上网了。
 *
 * 注意：热点用 APSTA 与 STA 共存；即使没有连上路由器也会开热点，
 * 这样在没有 WiFi 的场合仍然可以用手机遥控。
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
