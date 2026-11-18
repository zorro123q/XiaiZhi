#include "visual_follow.h"

#include <esp_camera.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <atomic>
#include <cmath>
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
constexpr int kGridCells = kGrid * kGrid;

// 目标区域的取法：从最密那一格出发做 8 邻接洪水填充，把连成一片的同色格全并进来。
//
// ⚠️ 这里以前是"最密格周围固定 7x7 格"（105x105 像素）。那个窗口最大只能罩住
// 11025 像素 = 画面的 19.1%，于是物体一靠近，面积就饱和在这个值上不再增长：
//   · `area_permille > 500`（糊满半屏就撤退）那条判据需要 50%，永远进不去，是死代码；
//   · 距离环在高位彻底失效，只能给一个恒定的小速度硬推，表现为"近了就猛退"。
// 改成洪水填充之后面积能一路长到接近满屏，高位才有分辨率可言。
//
// 为什么不是"全画面按色累加"：画面另一头如果有一块同色干扰（红椅子、红衣服），
// 质心会被拉到两者中间的空白处，车就会朝空地开。洪水填充只吃和种子连通的格，
// 隔开的干扰进不来 —— 既保留了旧窗口的抗干扰性，又没有它的尺寸上限。
constexpr int kCellSeedPixels = 2;  // 一格至少 2 个命中像素才认作连通，挡住零散噪点
// 连通格数上限（192/256 格）。整幅画面都是同一颜色时（对着红墙、红地板），
// 洪水填充会吞掉全屏、质心跑到中间空地 —— 卡个上限把面积的增长截在 75%，
// 高位仍然可用，同时不会因为"画面里大部分都是这个颜色"就以为自己贴脸了。
constexpr int kRegionMaxCells = kGridCells * 3 / 4;

// 少于这么多像素当作没看见。原来设 45（直径约 8px），目标稍一拉远就判丢、
// 于是停车不动 —— 实测体感"稍微远一点就不跟了"。降到 24 约合直径 5.5px，
// 噪点由 IsVivid 的饱和度门和 best_cnt>=4 的成块要求双重兜着。
constexpr int kMinPixels = 24;
// 跟丢多久就彻底退出。⚠️ 丢目标期间一律停车，绝不原地转着找：
// 底盘是开环的，车在原地扫视时用户往往正站在旁边，这个动作既吓人又容易撞。
constexpr int kLostExitMs = 2000;
// 每轮额外睡这么久。原来设 66ms（想卡 15fps），但 esp_camera_fb_get() 本身
// 就会阻塞到有帧，两段加起来实际只有 7fps 左右，转向响应慢半拍。
constexpr int kLoopPeriodMs = 10;
constexpr int kTaskStack = 5120;
// ⚠️ 必须是 2，不能更高。AFE 的取数任务（audio_communication）优先级是 3，
// audio_output 是 4，audio_input 是 8。设成 4 的话本任务和 audio_output 同级、
// 高于 AFE 取数任务，而扫一帧要在不让出 CPU 的情况下跑好几毫秒，
// AFE 的 ringbuffer 就会被喂满 —— 表现是串口刷
// "AFE: Ringbuffer of AFE(FEED) is full"。降到 2 之后，任何音频任务都能抢占它。
constexpr int kTaskPrio = 2;

// ── 距离环（前后）───────────────────────────────────────────────────
// 核心：用"等效直径"而不是"面积"当距离代理量。
//   小孔成像里像高 h = f*H/d，正比于 1/d；面积正比于 1/d²。
// 旧实现拿面积直接乘线性增益 = 把 1/d² 当 1/d 用，后果是近处一点点靠近就
// 给出极大速度、稍远一点又几乎不动，等于"越跟越抽"。
// 开方之后 size ∝ 1/d，误差与距离成正比，一个固定增益就能在全量程上一致：
//   size_ratio = sqrt(当前面积 / 目标面积) = 目标距离 / 当前距离
//   < 1 → 太远，前进；> 1 → 太近，后退；= 1 → 正好停住
constexpr float kSizeFilterAlpha = 0.45f;  // 一阶低通，压住逐帧面积噪声（15fps 下时间常数约 170ms）
// 相对死区。旧实现是"面积的绝对千分比 10"，对 8% 目标等于 ±12.5%、对 20% 目标只有 ±5%，
// 跟得越近死区越小、越容易抽。改成 size_ratio 上的相对量，距离带宽恒为 ±10%。
constexpr float kSizeDeadband = 0.10f;
// 相对距离误差 -> 速度。0.167 的误差（目标近了两成）给出 0.30 的速度。
constexpr float kDistGain = 1.8f;
// 每帧速度最多变化这么多。面积偶尔跳变（目标被手遮一下、走出光照区）时
// 不让车从 +0.75 直接翻到 -0.75 —— 那种翻转会把整车前后甩。
// 7fps 下 0.75 -> -0.75 需要 6 帧约 0.9 秒，仍在"物体扑过来能退开"的范围内。
constexpr float kYSlopePerFrame = 0.25f;
// 转向量大于这个值时前后速度按比例打折：先转身对准，别斜着冲过去。
// 车是开环的，斜着冲的时候转向基本不听使唤。
constexpr float kYawThrottleGain = 0.5f;

// 单遍扫描的累加桶。放静态区而不是栈上：这几块加起来近 3KB，
// 放 5KB 的任务栈里会把栈顶爆掉。
uint16_t s_cnt[kGrid * kGrid];
uint32_t s_sumx[kGrid * kGrid];
uint32_t s_sumy[kGrid * kGrid];
// 洪水填充的两个小数组。索引用 uint8_t 就够了（0~255 正好覆盖 256 格），
// 队列容量取格数上限，超出的格不再入队所以不会溢出。
uint8_t s_visited[kGrid * kGrid];
uint8_t s_queue[kGrid * kGrid];
int s_hue_hits[3];

struct State {
    TaskHandle_t task = nullptr;
    std::atomic<bool> running{false};
    std::atomic<bool> stop_req{false};
    Config cfg;
    MotionSink on_motion;
    ExitSink on_exit;
    FrameSink on_frame;

    // 对外的观测值，供状态查询和诊断
    std::atomic<int> frames{0};
    std::atomic<int> fps_milli{0};
    std::atomic<int> pixels{0};
    std::atomic<int> area_permille{0};
    // 等效直径相对目标的千分比（滤波后）。1000 = 距离正好，
    // >1000 = 目标比期望大（离太近），<1000 = 离太远。
    // 调距离参数时看这一个数就够，比看 area_pm 直观得多。
    std::atomic<int> size_permille{1000};
    std::atomic<int> bbox_w_permille{0};
    std::atomic<int> cells{0};
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
// 注意：调用方必须已经 bswap 过。扫帧和写预览共用同一次转换，不重复转。
inline void DecodeBswapped(uint16_t p, int* r, int* g, int* b) {
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
    int cells;               // 连通格数，用来判断"是不是大片同色背景"
    int bbox_w_permille;     // 外接框宽度占画面宽度的千分比
};

// 单遍扫描整帧：命中的像素按 15x15 像素归到 16x16 的格子里，同时累加
// 格内像素的坐标和。扫完就知道哪一格最密，再把那一格周围 7x7 格加起来，
// 一次得到精确质心和总面积，不用扫第二遍。
//
// preview 非空时顺手把 bswap 后的像素写进去，屏幕上就能同步看到画面。
// 复用同一次字节序转换，比事后再整帧拷一遍省一半时间。
bool Analyze(const uint8_t* buf, TargetHue hue, uint16_t* preview, Detection* out) {
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
        // 每 16 行让出一次 CPU。本任务优先级已经低于全部音频任务，
        // 但和 opus 编码（2）、骰子渲染（2）同级，长时间独占会让它们卡顿。
        if ((y & 15) == 0) {
            taskYIELD();
        }
        const int gy = y / kCell;
        const uint16_t* row = src + y * kFrameW;
        uint16_t* prow = (preview != nullptr) ? (preview + y * kFrameW) : nullptr;
        uint16_t* cnt_row = s_cnt + gy * kGrid;
        uint32_t* sx_row = s_sumx + gy * kGrid;
        uint32_t* sy_row = s_sumy + gy * kGrid;
        for (int x = 0; x < kFrameW; x++) {
            const uint16_t p = __builtin_bswap16(row[x]);
            if (prow != nullptr) {
                prow[x] = p;
            }
            int r = 0;
            int g = 0;
            int b = 0;
            int dominant = 0;
            DecodeBswapped(p, &r, &g, &b);
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

    // 从最密格出发做 8 邻接洪水填充，把连成一片的同色格全并进来。
    // 用显式队列而不是递归：16x16 的递归深度能到 256 层，5KB 任务栈撑不住。
    memset(s_visited, 0, sizeof(s_visited));
    int head = 0;
    int tail = 0;
    s_queue[tail++] = static_cast<uint8_t>(best_idx);
    s_visited[best_idx] = 1;

    uint32_t total = 0;
    uint32_t sumx = 0;
    uint32_t sumy = 0;
    int cells = 0;
    int gx_min = kGrid;
    int gx_max = -1;
    int gy_min = kGrid;
    int gy_max = -1;

    while (head < tail) {
        const int idx = s_queue[head++];
        const int gx = idx % kGrid;
        const int gy = idx / kGrid;
        total += s_cnt[idx];
        sumx += s_sumx[idx];
        sumy += s_sumy[idx];
        cells++;
        if (gx < gx_min) gx_min = gx;
        if (gx > gx_max) gx_max = gx;
        if (gy < gy_min) gy_min = gy;
        if (gy > gy_max) gy_max = gy;

        for (int dy = -1; dy <= 1; dy++) {
            const int ny = gy + dy;
            if (ny < 0 || ny >= kGrid) {
                continue;
            }
            for (int dx = -1; dx <= 1; dx++) {
                const int nx = gx + dx;
                if (nx < 0 || nx >= kGrid) {
                    continue;
                }
                const int ni = ny * kGrid + nx;
                if (s_visited[ni]) {
                    continue;
                }
                if (s_cnt[ni] < kCellSeedPixels) {
                    continue;
                }
                s_visited[ni] = 1;
                // 到上限就不再扩张。注意 visited 已经置位，这些格后面不会被
                // 重复检查，也不会被计入——面积就停在 75% 画面，不会被红墙撑爆。
                if (tail >= kRegionMaxCells) {
                    continue;
                }
                s_queue[tail++] = static_cast<uint8_t>(ni);
            }
        }
    }

    if (total < static_cast<uint32_t>(kMinPixels)) {
        return false;
    }

    out->pixels = static_cast<int>(total);
    out->cx_permille = static_cast<int>(sumx * 1000ULL / total / kFrameW);
    out->cy_permille = static_cast<int>(sumy * 1000ULL / total / kFrameH);
    out->cells = cells;
    out->bbox_w_permille = (gx_max - gx_min + 1) * kCell * 1000 / kFrameW;
    return true;
}

void TaskEntry(void* arg) {
    const Config cfg = s.cfg;
    const MotionSink on_motion = s.on_motion;
    const ExitSink on_exit = s.on_exit;
    const FrameSink on_frame = s.on_frame;

    const int64_t t0 = esp_timer_get_time();
    int lost = 0;
    int frames = 0;
    int fps_frames = 0;
    int64_t fps_t0 = t0;
    float prev_cx = 0.0f;
    bool warned_format = false;
    bool warned_region = false;
    int fb_fail = 0;
    int64_t lost_start_us = 0;
    // 滤波后的"目标距离 / 当前距离"。1.0 表示距离正好。
    float size_f = 1.0f;
    // 首次捕获（含丢失后重新捕获）时要直接把原始值灌进滤波器，
    // 否则它会从上一轮的旧值慢慢爬过来，前几帧的距离判断是错的。
    bool size_valid = false;
    // 斜坡限幅的基准：上一帧真正发出去的前后速度
    float prev_y_sent = 0.0f;

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
            found = Analyze(fb->buf, cfg.hue, cfg.preview_buffer, &det);
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

        // 预览：像素在上面 Analyze() 里已经顺手填好了，这里只负责按节流刷屏。
        // 从第一帧就刷（frames 是 1 起），让用户一启动就能看到画面。
        if (on_frame && cfg.preview_buffer != nullptr && cfg.preview_every_n_frames > 0 &&
            ((frames - 1) % cfg.preview_every_n_frames) == 0) {
            on_frame();
        }

        if (!found) {
            lost++;
            s.lost.store(lost);
            s.pixels.store(0);
            s.area_permille.store(0);
            // -1 表示"这一帧没有测量值"，跟"目标小到 0"区分开
            s.size_permille.store(-1);
            s.bbox_w_permille.store(0);
            s.cells.store(0);
            s.cx_permille.store(500);
            s.out_x_milli.store(0);
            s.out_y_milli.store(0);
            // 丢目标时把三处积分/滤波状态一起清掉。特别是 prev_cx：
            // 不清的话重新捕获的第一帧 cx-prev_cx 是个大跳变，kd 项会把车猛甩一下。
            size_valid = false;
            prev_cx = 0.0f;
            prev_y_sent = 0.0f;
            // 丢目标一律原地停车。⚠️ 不要改成"原地转着找"——底盘是开环的，
            // 车在原地扫视时用户往往正站在旁边，既吓人又容易撞，
            // 而且用户会理解为"目标都没了它还在自己动"。
            on_motion(0.0f, 0.0f);
            if (lost_start_us == 0) {
                lost_start_us = now;
                ESP_LOGW(TAG, "目标丢失，停车等待 %d 毫秒（这期间只在原地等，不移动）", kLostExitMs);
            } else if (now - lost_start_us > static_cast<int64_t>(kLostExitMs) * 1000) {
                ESP_LOGI(TAG, "目标已丢失 %d 毫秒，结束跟随", kLostExitMs);
                break;
            }
        } else {
            if (lost_start_us != 0) {
                ESP_LOGI(TAG, "重新捕获目标：%d 像素，此前丢失 %d 毫秒",
                         det.pixels, static_cast<int>((now - lost_start_us) / 1000));
            }
            lost_start_us = 0;
            lost = 0;
            s.lost.store(0);

            // 质心归一化到 [-1,1]：0 是画面正中，正数表示目标偏右
            const float cx = det.cx_permille / 500.0f - 1.0f;
            const int area_permille = det.pixels * 1000 / (kFrameW * kFrameH);

            // ── 转向（左右）──
            float x = cfg.kp * cx + cfg.kd * (cx - prev_cx);
            prev_cx = cx;
            // 转向死区：目标基本在正中时别左右抽。kp 提到 1.6 后，偏画面 1/13
            // 就会给出 0.12 的转向量，没有死区车会一直小幅摆。
            if (x < 0.12f && x > -0.12f) {
                x = 0.0f;
            }
            x = Clamp(x, -1.0f, 1.0f);

            // ── 距离（前后）──
            // 这一节解决的就是"物体靠近后退、远离前进"。
            //
            // 关键是把面积开方成"等效直径"再用：
            //   像高 h = f*H/d 正比于 1/d，面积正比于 1/d²。
            //   size_ratio = sqrt(当前面积 / 目标面积) = 目标距离 / 当前距离
            // 所以 size_ratio 就是"距离的相对倒数"：>1 太近、<1 太远、=1 正好。
            // 直接拿面积做线性闭环等于把 1/d² 当 1/d 用，近处一点点靠近就
            // 给出猛速度、远处几乎不动 —— 那是旧实现"走近就猛退、站远就不追"的根因。
            float area_tgt = cfg.target_area * static_cast<float>(kFrameW * kFrameH);
            if (area_tgt < 1.0f) {
                area_tgt = 1.0f;  // 除零保护；target_area 由工具限制在 3%~25%，正常进不来
            }
            const float size_raw = sqrtf(static_cast<float>(det.pixels) / area_tgt);
            if (!size_valid) {
                size_f = size_raw;
                size_valid = true;
            } else {
                size_f += kSizeFilterAlpha * (size_raw - size_f);
            }

            // 相对死区：距离偏差在 ±10% 以内就不动，防止在目标距离上来回抽。
            // 旧实现用的是"面积的绝对千分比"，跟得越近死区相对越小，越容易抽。
            const float dist_err = size_f - 1.0f;
            float y = 0.0f;
            if (dist_err > kSizeDeadband || dist_err < -kSizeDeadband) {
                // 减掉死区再算，否则"刚越过死区"那一帧会凭空跳一个台阶出来
                const float e = (dist_err > 0.0f) ? (dist_err - kSizeDeadband)
                                                  : (dist_err + kSizeDeadband);
                y = -kDistGain * e;
            }
            // 斜着冲的时候收油门：转向量大的时候先把头对准，别一边拐一边顶上去。
            // 底盘是开环的，边转边走实际路径全是弧线，很容易绕到目标侧后方。
            y *= (1.0f - kYawThrottleGain * (x < 0.0f ? -x : x));
            // 斜坡限幅：面积偶发跳变（手遮一下、进出光照区）时别让速度硬翻。
            // 真被目标逼近时 0.75 -> -0.75 大约要 6 帧（7fps 下 0.9 秒），还退得开。
            // ⚠️ 顺序不能反：斜坡必须在 clamp 之前。反过来的话 prev=0.60、目标 0.75 时
            // 斜坡会给出 0.85，直接超过 max_speed 往上顶。
            const float dy = y - prev_y_sent;
            if (dy > kYSlopePerFrame) {
                y = prev_y_sent + kYSlopePerFrame;
            } else if (dy < -kYSlopePerFrame) {
                y = prev_y_sent - kYSlopePerFrame;
            }
            y = Clamp(y, -cfg.max_speed, cfg.max_speed);
            prev_y_sent = y;

            // 连通区顶到上限 = 画面里有一大片同色（对着红墙、红地板、红衣服）。
            // 这时"面积"已经不代表距离了，只提示一次，具体照着跟不跟交给用户判断。
            if (det.cells >= kRegionMaxCells && !warned_region) {
                warned_region = true;
                ESP_LOGW(TAG, "同色区域占满 %d 格上限，画面里可能有大片同色背景，距离判断会失准",
                         kRegionMaxCells);
            }

            s.pixels.store(det.pixels);
            s.area_permille.store(area_permille);
            s.size_permille.store(static_cast<int>(size_f * 1000.0f));
            s.bbox_w_permille.store(det.bbox_w_permille);
            s.cells.store(det.cells);
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
    s.on_frame = nullptr;
    s.task = nullptr;
    s.running.store(false);
    ESP_LOGI(TAG, "跟随任务结束，共处理 %d 帧", frames);
    vTaskDelete(nullptr);
}

}  // namespace

bool Start(const Config& cfg, MotionSink on_motion, ExitSink on_exit, FrameSink on_frame) {
    if (s.running.load()) {
        return false;
    }
    s.cfg = cfg;
    s.on_motion = std::move(on_motion);
    s.on_exit = std::move(on_exit);
    s.on_frame = std::move(on_frame);
    s.stop_req.store(false);
    s.frames.store(0);
    s.fps_milli.store(0);
    s.pixels.store(0);
    s.area_permille.store(0);
    s.size_permille.store(1000);
    s.bbox_w_permille.store(0);
    s.cells.store(0);
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
        s.on_frame = nullptr;
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
    char buf[384];
    // 全程整数千分比输出：CONFIG_NEWLIB_NANO_FORMAT 下 printf 不支持 %f
    // size_pm 是诊断"为什么不前进/不后退"的关键字段：1000 表示距离正好，
    // 一直贴着 1000 却不走说明 PID 参数没问题而是别的地方（比如速度太小推不动）。
    // -1 表示这一帧没测到目标。
    snprintf(buf, sizeof(buf),
             "{\"running\":%s,\"frames\":%d,\"fps_x10\":%d,\"pixels\":%d,\"area_pm\":%d,"
             "\"size_pm\":%d,\"bbox_pm\":%d,\"cells\":%d,"
             "\"cx_pm\":%d,\"out_x_pm\":%d,\"out_y_pm\":%d,\"lost\":%d,\"elapsed_ms\":%d,"
             "\"hue_hits\":[%d,%d,%d]}",
             s.running.load() ? "true" : "false",
             s.frames.load(),
             s.fps_milli.load() / 100,
             s.pixels.load(),
             s.area_permille.load(),
             s.size_permille.load(),
             s.bbox_w_permille.load(),
             s.cells.load(),
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
