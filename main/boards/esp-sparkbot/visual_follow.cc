#include "visual_follow.h"

#include <esp_camera.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <atomic>
#include <cstdio>
#include <cstring>

#define TAG "VisualFollow"

namespace vfollow {
namespace {

// 摄像头在 esp_sparkbot_board.cc 的 InitializeCamera() 里固定配成
// RGB565 / 240x240。这里按同一套尺寸做桶累加，尺寸对不上就丢弃该帧，
// 不做缩放——缩放在 S3 上比直接判色还贵。
constexpr int kFrameW = 240;
constexpr int kFrameH = 240;
constexpr int kGrid = 16;
constexpr int kCell = kFrameW / kGrid;  // 15 像素一格
// 质心只取最密那格周围 (2kNeighbor+1)^2 格。取太大等于全画面求质心，
// 画面另一端有一块同色干扰就会把中心拉到中间空白处；取 7x7 格合 105x105 像素，
// 够罩住一个占到 19% 画面的球。
constexpr int kNeighbor = 3;

constexpr int kMinPixels = 45;          // 少于这么多像素当作没看见，滤掉噪点
constexpr int kLostStopFrames = 6;      // 刚跟丢先停 6 帧（约 0.4 秒），别立刻乱转
constexpr int kSearchFrames = 120;      // 再原地搜 120 帧（约 8 秒）才彻底放弃
constexpr int kLoopPeriodMs = 66;       // 上层目标 15fps，实际帧率受摄像头限制
constexpr int kTaskStack = 5120;
constexpr int kTaskPrio = 4;            // 低于音频任务，高于显示任务
constexpr int kAreaDeadbandMilli = 15;  // 面积死区 1.5%，防止原地前后抽动
constexpr float kAreaGain = 6.0f;       // 面积误差 -> 速度的增益

// 单遍扫描的累加桶。放静态区而不是栈上：这三块加起来 2.5KB，
// 放 5KB 的任务栈里会把栈顶爆掉。
uint16_t s_cnt[kGrid * kGrid];
uint32_t s_sumx[kGrid * kGrid];
uint32_t s_sumy[kGrid * kGrid];
int s_hue_hits[3];

struct State {
    TaskHandle_t task = nullptr;
    std::atomic<bool> running{false};
    std::atomic<bool> stop_req{false};
    Config cfg;
    MotionSink on_motion;
    ExitSink on_exit;

    // 对外的观测值，供状态查询和诊断
    std::atomic<int> frames{0};
    std::atomic<int> fps_milli{0};
    std::atomic<int> pixels{0};
    std::atomic<int> area_permille{0};
    std::atomic<int> cx_permille{500};
    std::atomic<int> out_x_milli{0};
    std::atomic<int> out_y_milli{0};
    std::atomic<int> lost{0};
    std::atomic<int> elapsed_ms{0};
    std::atomic<int> hue_hit_red{0};
    std::atomic<int> hue_hit_green{0};
    std::atomic<int> hue_hit_blue{0};
};

State s;

inline float Clamp(float v, float lo, float hi) {
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

// 传感器吐的是大端 RGB565：Capture() 里也是先 __builtin_bswap16 再喂给 LVGL 的。
// 这里保持同一套字节序，识别到的颜色才和用户在预览里看到的一致，
// 否则红蓝会对调，表现为"明明是红球却跟不动"。
inline void DecodeRgb565(uint16_t raw, int* r, int* g, int* b) {
    const uint16_t p = __builtin_bswap16(raw);
    *r = static_cast<int>((p >> 11) & 0x001F) * 255 / 31;
    *g = static_cast<int>((p >> 5) & 0x003F) * 255 / 63;
    *b = static_cast<int>(p & 0x001F) * 255 / 31;
}

// 与色相无关的"够亮 + 够艳"筛选：灰墙、白墙、肤色、暗部全在这里挡掉，
// 后面的色相判定只处理真正有颜色的像素。dominant 输出主色通道 0/1/2。
inline bool IsVivid(int r, int g, int b, int* dominant) {
    int mx = r > g ? r : g;
    if (b > mx) {
        mx = b;
    }
    if (mx < 55) {
        return false;
    }
    int mn = r < g ? r : g;
    if (b < mn) {
        mn = b;
    }
    // 饱和度 (mx-mn)/mx >= 0.41，用整数乘法避免浮点和除法
    if ((mx - mn) * 255 < 105 * mx) {
        return false;
    }
    *dominant = (mx == r) ? 0 : ((mx == g) ? 1 : 2);
    return true;
}

inline bool MatchesHue(int r, int g, int b, int dominant, TargetHue hue) {
    switch (hue) {
        case TargetHue::kRed:
            // 红通道只有 5 位（步长 8），通道差得放宽一点，否则暗红判不出来
            return dominant == 0 && r - g > 38 && r - b > 38;
        case TargetHue::kGreen:
            // 绿是 6 位（步长 4），可以判得严一些，顺带避开黄绿偏暖的光源
            return dominant == 1 && g - r > 34 && g - b > 22;
        case TargetHue::kBlue:
            return dominant == 2 && b - r > 38 && b - g > 30;
    }
    return false;
}

struct Detection {
    int pixels;
    int cx_permille;  // 质心 x，千分比 [0,1000]
    int cy_permille;
};

// 单遍扫描整帧：命中的像素按 15x15 像素归到 16x16 的格子里，同时累加
// 格内像素的坐标和。扫完就知道哪一格最密，再把那一格周围 7x7 格加起来，
// 一次得到精确质心和总面积，不用扫第二遍。
bool Analyze(const uint8_t* buf, TargetHue hue, Detection* out) {
    if (buf == nullptr) {
        return false;
    }
    memset(s_cnt, 0, sizeof(s_cnt));
    memset(s_sumx, 0, sizeof(s_sumx));
    memset(s_sumy, 0, sizeof(s_sumy));
    s_hue_hits[0] = 0;
    s_hue_hits[1] = 0;
    s_hue_hits[2] = 0;

    const uint16_t* src = reinterpret_cast<const uint16_t*>(buf);
    for (int y = 0; y < kFrameH; y++) {
        const int gy = y / kCell;
        const uint16_t* row = src + y * kFrameW;
        uint16_t* cnt_row = s_cnt + gy * kGrid;
        uint32_t* sx_row = s_sumx + gy * kGrid;
        uint32_t* sy_row = s_sumy + gy * kGrid;
        for (int x = 0; x < kFrameW; x++) {
            int r = 0;
            int g = 0;
            int b = 0;
            int dominant = 0;
            DecodeRgb565(row[x], &r, &g, &b);
            if (!IsVivid(r, g, b, &dominant)) {
                continue;
            }
            s_hue_hits[dominant]++;
            if (!MatchesHue(r, g, b, dominant, hue)) {
                continue;
            }
            const int gx = x / kCell;
            cnt_row[gx]++;
            sx_row[gx] += static_cast<uint32_t>(x);
            sy_row[gx] += static_cast<uint32_t>(y);
        }
    }

    int best_cnt = 0;
    int best_idx = 0;
    for (int i = 0; i < kGrid * kGrid; i++) {
        if (s_cnt[i] > best_cnt) {
            best_cnt = s_cnt[i];
            best_idx = i;
        }
    }
    // 最密的格子还不到 4 个像素，说明整幅画面里没有成块的目标
    if (best_cnt < 4) {
        return false;
    }

    const int bx = best_idx % kGrid;
    const int by = best_idx / kGrid;
    uint32_t total = 0;
    uint32_t sumx = 0;
    uint32_t sumy = 0;
    for (int gy = by - kNeighbor; gy <= by + kNeighbor; gy++) {
        if (gy < 0 || gy >= kGrid) {
            continue;
        }
        for (int gx = bx - kNeighbor; gx <= bx + kNeighbor; gx++) {
            if (gx < 0 || gx >= kGrid) {
                continue;
            }
            const int i = gy * kGrid + gx;
            total += s_cnt[i];
            sumx += s_sumx[i];
            sumy += s_sumy[i];
        }
    }
    if (total < static_cast<uint32_t>(kMinPixels)) {
        return false;
    }

    out->pixels = static_cast<int>(total);
    out->cx_permille = static_cast<int>(sumx * 1000ULL / total / kFrameW);
    out->cy_permille = static_cast<int>(sumy * 1000ULL / total / kFrameH);
    return true;
}

void TaskEntry(void* arg) {
    const Config cfg = s.cfg;
    const MotionSink on_motion = s.on_motion;
    const ExitSink on_exit = s.on_exit;

    const int64_t t0 = esp_timer_get_time();
    int lost = 0;
    int frames = 0;
    int fps_frames = 0;
    int64_t fps_t0 = t0;
    float prev_cx = 0.0f;
    bool warned_format = false;
    int fb_fail = 0;

    while (!s.stop_req.load()) {
        const int elapsed_ms = static_cast<int>((esp_timer_get_time() - t0) / 1000);
        s.elapsed_ms.store(elapsed_ms);
        if (elapsed_ms > cfg.max_duration_ms) {
            ESP_LOGW(TAG, "跟随已跑满 %d 秒，自动停车", cfg.max_duration_ms / 1000);
            break;
        }

        camera_fb_t* fb = esp_camera_fb_get();
        if (fb == nullptr) {
            // 手机网页预览每 150ms 也会来抢一帧，驱动只有 2 个 buffer，
            // 偶尔抢不到是正常的。所以只在第 1/10/50 次失败时提示，别在 15fps 下刷屏。
            fb_fail++;
            if (fb_fail == 1 || fb_fail == 10 || fb_fail == 50) {
                ESP_LOGW(TAG, "取帧失败第 %d 次（网页预览和跟随共用摄像头缓冲）", fb_fail);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        fb_fail = 0;

        Detection det = {};
        bool found = false;
        // 除了尺寸和格式，还要卡住 fb->len：接下来是裸指针逐像素扫描，
        // 长度不够就会读过缓冲末尾，直接崩。Capture() 那边也是用 len/2 算像素数的。
        if (fb->format == PIXFORMAT_RGB565 &&
            static_cast<int>(fb->width) == kFrameW &&
            static_cast<int>(fb->height) == kFrameH &&
            fb->len >= static_cast<size_t>(kFrameW) * kFrameH * 2) {
            found = Analyze(fb->buf, cfg.hue, &det);
        } else if (!warned_format) {
            ESP_LOGE(TAG, "帧格式不符合预期：fmt=%d %dx%d len=%d（需要 RGB565 %dx%d 且 len>=%d）",
                     static_cast<int>(fb->format), static_cast<int>(fb->width),
                     static_cast<int>(fb->height), static_cast<int>(fb->len),
                     kFrameW, kFrameH, kFrameW * kFrameH * 2);
            warned_format = true;
        }
        esp_camera_fb_return(fb);

        frames++;
        fps_frames++;
        const int64_t now = esp_timer_get_time();
        if (now - fps_t0 >= 1000000) {
            s.fps_milli.store(static_cast<int>(fps_frames * 1000LL * 1000000 / (now - fps_t0)));
            fps_frames = 0;
            fps_t0 = now;
        }
        s.frames.store(frames);
        s.hue_hit_red.store(s_hue_hits[0]);
        s.hue_hit_green.store(s_hue_hits[1]);
        s.hue_hit_blue.store(s_hue_hits[2]);

        if (!found) {
            lost++;
            s.lost.store(lost);
            s.pixels.store(0);
            s.area_permille.store(0);
            if (lost < kLostStopFrames) {
                // 刚丢：先停，别让车按上一帧的方向继续冲
                on_motion(0.0f, 0.0f);
            } else if (lost < kLostStopFrames + kSearchFrames) {
                // 丢了一会儿：原地慢慢转着找。不前进，避免撞上丢之前那个方向的障碍
                on_motion(0.5f, 0.0f);
            } else {
                ESP_LOGI(TAG, "原地找了 %d 帧仍没找到目标，结束跟随", kSearchFrames);
                break;
            }
        } else {
            if (lost >= kLostStopFrames) {
                ESP_LOGI(TAG, "重新捕获目标，命中 %d 像素", det.pixels);
            }
            lost = 0;
            s.lost.store(0);

            // 质心归一化到 [-1,1]：0 是画面正中，正数表示目标偏右
            const float cx = det.cx_permille / 500.0f - 1.0f;
            const int area_permille = det.pixels * 1000 / (kFrameW * kFrameH);
            const int target_permille = static_cast<int>(cfg.target_area * 1000.0f);
            const int err_permille = target_permille - area_permille;

            float y = 0.0f;
            if (err_permille > kAreaDeadbandMilli || err_permille < -kAreaDeadbandMilli) {
                // err 是真千分比，先除回 0~1 再乘增益：目标占 8% 而当前只占 2% 时，
                // err=60 -> y=0.36，正好是"明显偏小就中速前进"的手感。
                y = static_cast<float>(err_permille) / 1000.0f * kAreaGain;
            }
            if (area_permille > 500) {
                // 目标已经糊满半个画面，贴脸了，无论面积误差怎么算都先撤
                y = -0.4f;
            }
            y = Clamp(y, -cfg.max_speed, cfg.max_speed);

            float x = cfg.kp * cx + cfg.kd * (cx - prev_cx);
            prev_cx = cx;
            x = Clamp(x, -1.0f, 1.0f);

            s.pixels.store(det.pixels);
            s.area_permille.store(area_permille);
            s.cx_permille.store(det.cx_permille);
            s.out_x_milli.store(static_cast<int>(x * 1000.0f));
            s.out_y_milli.store(static_cast<int>(y * 1000.0f));

            on_motion(x, y);
        }

        vTaskDelay(pdMS_TO_TICKS(kLoopPeriodMs));
    }

    // 收尾：先让调用方把车停稳，再清状态。
    // 顺序不能反 —— s.running 置 false 之后 Start() 就会放行新的任务，
    // 那时如果 on_motion 还被清空着，新任务会拿到空回调。
    on_motion(0.0f, 0.0f);
    if (on_exit) {
        on_exit();
    }
    s.on_motion = nullptr;
    s.on_exit = nullptr;
    s.task = nullptr;
    s.running.store(false);
    ESP_LOGI(TAG, "跟随任务结束，共处理 %d 帧", frames);
    vTaskDelete(nullptr);
}

}  // namespace

bool Start(const Config& cfg, MotionSink on_motion, ExitSink on_exit) {
    if (s.running.load()) {
        return false;
    }
    s.cfg = cfg;
    s.on_motion = std::move(on_motion);
    s.on_exit = std::move(on_exit);
    s.stop_req.store(false);
    s.frames.store(0);
    s.fps_milli.store(0);
    s.pixels.store(0);
    s.area_permille.store(0);
    s.cx_permille.store(500);
    s.out_x_milli.store(0);
    s.out_y_milli.store(0);
    s.lost.store(0);
    s.elapsed_ms.store(0);
    s.hue_hit_red.store(0);
    s.hue_hit_green.store(0);
    s.hue_hit_blue.store(0);
    s.running.store(true);

    if (xTaskCreate(TaskEntry, "visual_follow", kTaskStack, nullptr, kTaskPrio, &s.task) != pdPASS) {
        s.running.store(false);
        s.on_motion = nullptr;
        s.on_exit = nullptr;
        ESP_LOGE(TAG, "创建跟随任务失败（内存不足）");
        return false;
    }

    ESP_LOGI(TAG, "跟随启动：色相 %d，目标面积 %d%%，kp %d/100 kd %d/100，上限 %d 秒",
             static_cast<int>(cfg.hue),
             static_cast<int>(cfg.target_area * 100.0f),
             static_cast<int>(cfg.kp * 100.0f),
             static_cast<int>(cfg.kd * 100.0f),
             cfg.max_duration_ms / 1000);
    return true;
}

void Stop() {
    if (!s.running.load()) {
        return;
    }
    // 用 exchange 而不是 store：这个函数会被摇杆、StopMotion、工具回调反复调用，
    // 只在第一次真的置位时打日志，否则拖动摇杆那 150ms 一拍会刷一串一模一样的行。
    if (s.stop_req.exchange(true)) {
        return;
    }
    ESP_LOGI(TAG, "已请求停止跟随（任务会在下一帧退出）");
}

bool IsRunning() {
    return s.running.load();
}

std::string GetStatusJson() {
    char buf[288];
    // 全程整数千分比输出：CONFIG_NEWLIB_NANO_FORMAT 下 printf 不支持 %f
    snprintf(buf, sizeof(buf),
             "{\"running\":%s,\"frames\":%d,\"fps_x10\":%d,\"pixels\":%d,\"area_pm\":%d,"
             "\"cx_pm\":%d,\"out_x_pm\":%d,\"out_y_pm\":%d,\"lost\":%d,\"elapsed_ms\":%d,"
             "\"hue_hits\":[%d,%d,%d]}",
             s.running.load() ? "true" : "false",
             s.frames.load(),
             s.fps_milli.load() / 100,
             s.pixels.load(),
             s.area_permille.load(),
             s.cx_permille.load(),
             s.out_x_milli.load(),
             s.out_y_milli.load(),
             s.lost.load(),
             s.elapsed_ms.load(),
             s.hue_hit_red.load(),
             s.hue_hit_green.load(),
             s.hue_hit_blue.load());
    return std::string(buf);
}

}  // namespace vfollow
