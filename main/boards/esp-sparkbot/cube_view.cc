#include "cube_view.h"

#include <driver/i2c_master.h>
#include <esp_err.h>
#include <esp_heap_caps.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <cmath>

#include "display.h"
#include "lvgl.h"

#define TAG "cube"

namespace cubetoy {
namespace {

// ── 画面参数 ──────────────────────────────────────────────────────────
constexpr int kSize = 200;          // 画布边长（屏幕是 240x240，留 20px 边距）
constexpr int kHalf = kSize / 2;    // 100，既是画布中心也是投影的焦距基准
constexpr float kFovDeg = 35.0f;    // 视场角：越小透视越平、越不像鱼眼
constexpr float kDist = 6.0f;       // 相机到骰子中心的距离（以骰子半径为单位）
constexpr int kFrameMs = 40;        // 25fps。再快 SPI 也刷不动 200x200
constexpr float kSpinDegPerFrame = 0.5f;  // 自转速度 ≈12.5°/s，转一圈约 29 秒

const float kFocal = 1.0f / tanf(kFovDeg * 0.5f * (float)M_PI / 180.0f);  // ≈3.17

// ── 模块状态 ──────────────────────────────────────────────────────────
Display* s_display = nullptr;
lv_obj_t* s_root = nullptr;    // 全屏黑底容器（挂在 lv_layer_top 上）
lv_obj_t* s_canvas = nullptr;  // 把 RGB565 缓冲区当图片显示
uint16_t* s_buf = nullptr;     // kSize*kSize 的像素缓冲
TaskHandle_t s_task = nullptr;
volatile bool s_visible = false;

float s_pitch = 22.0f;  // 绕 X
float s_roll = 10.0f;   // 绕 Z
float s_yaw = 0.0f;     // 绕 Y，自转时递增

std::string s_probe_summary = "尚未扫描";

// ── 基础数学 ──────────────────────────────────────────────────────────
struct V3 {
    float x, y, z;
};

struct P2 {
    float x, y;   // 屏幕坐标
    float z;      // 视空间深度，用来排序
    float s;      // 该点的缩放系数（像素 / 骰子单位）
};

inline uint16_t Rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

// 依次绕 Y（yaw）→ X（pitch）→ Z（roll）旋转。
// 顺序定了就不能随便换，否则同一个姿态下骰子的朝向会跟 IMU 对不上。
V3 Rotate(const V3& v, float pitch, float roll, float yaw) {
    const float rx = pitch * (float)M_PI / 180.0f;
    const float ry = yaw * (float)M_PI / 180.0f;
    const float rz = roll * (float)M_PI / 180.0f;

    // Y
    float cy = cosf(ry), sy = sinf(ry);
    float x1 = v.x * cy + v.z * sy;
    float z1 = -v.x * sy + v.z * cy;
    // X
    float cx = cosf(rx), sx = sinf(rx);
    float y2 = v.y * cx - z1 * sx;
    float z2 = v.y * sx + z1 * cx;
    // Z
    float cz = cosf(rz), sz = sinf(rz);
    float x3 = x1 * cz - y2 * sz;
    float y3 = x1 * sz + y2 * cz;

    return {x3, y3, z2};
}

P2 Project(const V3& view) {
    float vz = kDist - view.z;             // 相机在 +z 方向，朝原点看
    if (vz < 0.5f) vz = 0.5f;              // 防除零
    float s = (kHalf * kFocal) / vz;
    return {kHalf + view.x * s, kHalf - view.y * s, vz, s};
}

// ── 软光栅 ────────────────────────────────────────────────────────────
void FillTri(const P2& a, const P2& b, const P2& c, uint16_t color) {
    int minx = (int)floorf(fminf(a.x, fminf(b.x, c.x)));
    int maxx = (int)ceilf(fmaxf(a.x, fmaxf(b.x, c.x)));
    int miny = (int)floorf(fminf(a.y, fminf(b.y, c.y)));
    int maxy = (int)ceilf(fmaxf(a.y, fmaxf(b.y, c.y)));
    if (minx < 0) minx = 0;
    if (miny < 0) miny = 0;
    if (maxx > kSize - 1) maxx = kSize - 1;
    if (maxy > kSize - 1) maxy = kSize - 1;

    const float d = (b.y - c.y) * (a.x - c.x) + (c.x - b.x) * (a.y - c.y);
    if (fabsf(d) < 1e-6f) return;  // 退化三角形（侧面看时会出现）

    for (int y = miny; y <= maxy; y++) {
        for (int x = minx; x <= maxx; x++) {
            const float px = x + 0.5f, py = y + 0.5f;
            const float w1 = ((b.y - c.y) * (px - c.x) + (c.x - b.x) * (py - c.y)) / d;
            const float w2 = ((c.y - a.y) * (px - c.x) + (a.x - c.x) * (py - c.y)) / d;
            const float w3 = 1.0f - w1 - w2;
            if (w1 >= 0.0f && w2 >= 0.0f && w3 >= 0.0f) {
                s_buf[y * kSize + x] = color;
            }
        }
    }
}

void FillQuad(const P2& a, const P2& b, const P2& c, const P2& d, uint16_t color) {
    FillTri(a, b, c, color);
    FillTri(a, c, d, color);
}

// 画一条 1px 的线，用来给每个面描边。不加边的话，明暗相近的两个面会糊成一片。
void DrawLine(const P2& a, const P2& b, uint16_t color) {
    float dx = b.x - a.x, dy = b.y - a.y;
    int steps = (int)fmaxf(fabsf(dx), fabsf(dy)) + 1;
    if (steps <= 0) return;
    for (int i = 0; i <= steps; i++) {
        float t = (float)i / (float)steps;
        int x = (int)(a.x + dx * t);
        int y = (int)(a.y + dy * t);
        if (x >= 0 && x < kSize && y >= 0 && y < kSize) {
            s_buf[y * kSize + x] = color;
        }
    }
}

// ── 骰子的几何 ────────────────────────────────────────────────────────
// 8 个顶点，以中心为原点、半边长=1
const V3 kVerts[8] = {
    {-1, 1, 1},  {1, 1, 1},  {1, -1, 1},  {-1, -1, 1},
    {-1, 1, -1}, {1, 1, -1}, {1, -1, -1}, {-1, -1, -1},
};

struct Face {
    int v[4];      // 四个顶点索引，按外面看是逆时针
    V3 normal;     // 外法线，用来做背面剔除和明暗
    V3 center;     // 面心
    V3 u, v;       // 面内两个正交方向，用来摆点数
    int pips;      // 点数 1~6
};

// 对面点数之和为 7（真骰子规则）：1-6、3-4、5-2
const Face kFaces[6] = {
    {{0, 1, 2, 3}, {0, 0, 1},  {0, 0, 1},  {1, 0, 0},  {0, 1, 0},  1},  // 前
    {{5, 4, 7, 6}, {0, 0, -1}, {0, 0, -1}, {-1, 0, 0}, {0, 1, 0},  6},  // 后
    {{1, 5, 6, 2}, {1, 0, 0},  {1, 0, 0},  {0, 0, -1}, {0, 1, 0},  3},  // 右
    {{4, 0, 3, 7}, {-1, 0, 0}, {-1, 0, 0}, {0, 0, 1},  {0, 1, 0},  4},  // 左
    {{4, 5, 1, 0}, {0, 1, 0},  {0, 1, 0},  {1, 0, 0},  {0, 0, -1}, 5},  // 上
    {{3, 2, 6, 7}, {0, -1, 0}, {0, -1, 0}, {1, 0, 0},  {0, 0, 1},  2},  // 下
};

constexpr float kPipOffset = 0.46f;  // 点数离面心的距离
constexpr float kPipRadius = 0.14f;

// 按点数把位置填进 out（面内坐标，范围约 ±0.46）
int PipLayout(int n, float out[6][2]) {
    const float c = kPipOffset;
    switch (n) {
        case 1:
            out[0][0] = 0;    out[0][1] = 0;
            return 1;
        case 2:
            out[0][0] = -c;   out[0][1] = -c;
            out[1][0] = c;    out[1][1] = c;
            return 2;
        case 3:
            out[0][0] = -c;   out[0][1] = -c;
            out[1][0] = 0;    out[1][1] = 0;
            out[2][0] = c;    out[2][1] = c;
            return 3;
        case 4:
            out[0][0] = -c;   out[0][1] = -c;
            out[1][0] = c;    out[1][1] = -c;
            out[2][0] = -c;   out[2][1] = c;
            out[3][0] = c;    out[3][1] = c;
            return 4;
        case 5:
            out[0][0] = -c;   out[0][1] = -c;
            out[1][0] = c;    out[1][1] = -c;
            out[2][0] = 0;    out[2][1] = 0;
            out[3][0] = -c;   out[3][1] = c;
            out[4][0] = c;    out[4][1] = c;
            return 5;
        default:
            out[0][0] = -c;   out[0][1] = -c;
            out[1][0] = c;    out[1][1] = -c;
            out[2][0] = -c;   out[2][1] = 0;
            out[3][0] = c;    out[3][1] = 0;
            out[4][0] = -c;   out[4][1] = c;
            out[5][0] = c;    out[5][1] = c;
            return 6;
    }
}

// 渲染一帧到 s_buf
void Render() {
    memset(s_buf, 0, kSize * kSize * sizeof(uint16_t));  // 黑底

    // 光源方向（视空间），大致从相机左上打过来
    const V3 light = {-0.35f, 0.55f, 0.76f};

    // 先把顶点和法线都转到视空间，一次算完复用
    V3 view_v[8];
    for (int i = 0; i < 8; i++) {
        view_v[i] = Rotate(kVerts[i], s_pitch, s_roll, s_yaw);
    }

    struct Visible {
        int face;
        float depth;
    } vis[6];
    int vis_count = 0;

    for (int f = 0; f < 6; f++) {
        V3 n = Rotate(kFaces[f].normal, s_pitch, s_roll, s_yaw);
        V3 c = Rotate(kFaces[f].center, s_pitch, s_roll, s_yaw);
        // 相机在 (0,0,+kDist) 朝 -z 看，所以"相机→面心"是 c-(0,0,kDist)。
        // 外法线与这个向量夹角大于 90°（点积为负）才朝向相机。
        float cx = c.x, cy = c.y, cz = c.z - kDist;
        if (n.x * cx + n.y * cy + n.z * cz >= 0.0f) {
            continue;  // 背面，剔除
        }
        vis[vis_count].face = f;
        vis[vis_count].depth = kDist - c.z;  // 越大越远
        vis_count++;
    }

    // 远的先画（凸多面体 + 背面剔除后其实不会互相遮挡，排序只是保险）
    for (int i = 0; i < vis_count; i++) {
        for (int j = i + 1; j < vis_count; j++) {
            if (vis[j].depth > vis[i].depth) {
                Visible t = vis[i];
                vis[i] = vis[j];
                vis[j] = t;
            }
        }
    }

    for (int k = 0; k < vis_count; k++) {
        const Face& face = kFaces[vis[k].face];

        V3 n = Rotate(face.normal, s_pitch, s_roll, s_yaw);
        float lambert = n.x * light.x + n.y * light.y + n.z * light.z;
        if (lambert < 0.0f) lambert = 0.0f;
        // 环境光 0.42 + 漫反射 0.58：侧脸不至于全黑，又能看出三个面的明暗差
        float shade = 0.42f + 0.58f * lambert;
        uint8_t g = (uint8_t)(235.0f * shade);
        uint16_t face_color = Rgb565(g, g, (uint8_t)fminf(255.0f, g * 1.02f));

        P2 p[4];
        for (int i = 0; i < 4; i++) {
            p[i] = Project(view_v[face.v[i]]);
        }
        FillQuad(p[0], p[1], p[2], p[3], face_color);

        // 描边：相邻面明暗接近，没有边线会看成一整块
        const uint16_t edge_color = Rgb565(70, 70, 76);
        for (int i = 0; i < 4; i++) {
            DrawLine(p[i], p[(i + 1) % 4], edge_color);
        }

        // 点数同样是"先算面内 3D 坐标，再走同一条变换链"，所以它会跟着面一起透视压缩。
        // 用 12 边形近似圆：直接画屏幕空间的圆在斜面上会变成胖椭圆，很假。
        float pos[6][2];
        int pip_count = PipLayout(face.pips, pos);
        const uint16_t pip_color = Rgb565(22, 22, 28);
        for (int i = 0; i < pip_count; i++) {
            V3 ctr = {
                face.center.x + face.u.x * pos[i][0] + face.v.x * pos[i][1],
                face.center.y + face.u.y * pos[i][0] + face.v.y * pos[i][1],
                face.center.z + face.u.z * pos[i][0] + face.v.z * pos[i][1],
            };
            P2 center_p = Project(Rotate(ctr, s_pitch, s_roll, s_yaw));
            if (kPipRadius * center_p.s < 0.7f) {
                continue;  // 太小了，画出来只是噪点
            }

            P2 ring[12];
            for (int k2 = 0; k2 < 12; k2++) {
                float ang = 2.0f * (float)M_PI * k2 / 12.0f;
                float du = kPipRadius * cosf(ang);
                float dv = kPipRadius * sinf(ang);
                V3 pt = {
                    ctr.x + face.u.x * du + face.v.x * dv,
                    ctr.y + face.u.y * du + face.v.y * dv,
                    ctr.z + face.u.z * du + face.v.z * dv,
                };
                ring[k2] = Project(Rotate(pt, s_pitch, s_roll, s_yaw));
            }
            for (int k2 = 0; k2 < 12; k2++) {
                FillTri(center_p, ring[k2], ring[(k2 + 1) % 12], pip_color);
            }
        }
    }
}

// ── 后台渲染任务 ──────────────────────────────────────────────────────
// 常驻任务而不是每次显隐都建/删：建删会造成 Show 抢在任务退出的瞬间，
// 结果画面卡在最后一帧。不显示的时候它只是每 100ms 醒一次，几乎不耗 CPU。
void RenderTask(void*) {
    while (true) {
        if (!s_visible || s_display == nullptr) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        {
            // 所有 LVGL 调用必须在锁里做，否则会和 UI 任务抢 lvgl 内部状态
            DisplayLockGuard lock(s_display);
            s_yaw += kSpinDegPerFrame;  // 第 1 轮固定自转；接上 IMU 后改由 SetAttitude 驱动
            if (s_yaw > 360.0f) s_yaw -= 360.0f;
            Render();
            lv_obj_invalidate(s_canvas);
        }
        vTaskDelay(pdMS_TO_TICKS(kFrameMs));
    }
}

}  // namespace

// ── I2C 扫描：确认 IMU 到底是哪颗 ────────────────────────────────────
void ProbeImu(i2c_master_bus_handle_t bus, Display* display) {
    (void)display;
    if (bus == nullptr) {
        s_probe_summary = "I2C 总线未初始化";
        return;
    }

    char buf[320] = {0};
    size_t used = 0;

    ESP_LOGI(TAG, "开始扫描 I2C 总线（0x08~0x77）");
    for (uint16_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_master_probe(bus, addr, 50) != ESP_OK) {
            continue;
        }
        uint8_t chip_id = 0xFF;
        bool id_ok = false;

        i2c_device_config_t cfg = {};
        cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        cfg.device_address = addr;
        cfg.scl_speed_hz = 100000;
        i2c_master_dev_handle_t dev = nullptr;
        if (i2c_master_bus_add_device(bus, &cfg, &dev) == ESP_OK) {
            uint8_t reg = 0x00;
            if (i2c_master_transmit_receive(dev, &reg, 1, &chip_id, 1, 100) == ESP_OK) {
                id_ok = true;
            }
            i2c_master_bus_rm_device(dev);
        }

        ESP_LOGI(TAG, "  发现设备 0x%02X，0x00 寄存器=%s0x%02X", addr,
                 id_ok ? "" : "(读不到) ", chip_id);

        int n = 0;
        if (used + 20 < sizeof(buf)) {  // 留余量，防止 used 溢出后 strlen 越界
            n = snprintf(buf + used, sizeof(buf) - used, "%s0x%02X=0x%02X",
                         used ? " " : "", addr, id_ok ? chip_id : 0xFF);
            if (n > 0) used += (size_t)n;
        }
    }
    ESP_LOGI(TAG, "I2C 扫描结束");

    if (used == 0) {
        s_probe_summary = "总线上没有任何设备应答（连 ES8311 都没有，检查 I2C 是否初始化）";
    } else {
        s_probe_summary = std::string(buf) + "　（0x18=ES8311 音频；0x68/0x69=BMI270 系；0x12/0x13=QMA 系；0x6A/0x6B=QMI8658）";
    }
    ESP_LOGI(TAG, "扫描摘要: %s", s_probe_summary.c_str());
}

std::string GetProbeSummary() {
    return s_probe_summary;
}

bool Init(Display* display) {
    if (display == nullptr) {
        ESP_LOGE(TAG, "display 为空，无法创建立方体");
        return false;
    }
    s_display = display;

    // 像素缓冲放 PSRAM：80KB，内部 RAM 留着给音频和 LVGL 用
    s_buf = (uint16_t*)heap_caps_malloc(kSize * kSize * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
    if (s_buf == nullptr) {
        ESP_LOGW(TAG, "PSRAM 分配失败，退回内部 RAM");
        s_buf = (uint16_t*)heap_caps_malloc(kSize * kSize * sizeof(uint16_t), MALLOC_CAP_8BIT);
    }
    if (s_buf == nullptr) {
        ESP_LOGE(TAG, "缓冲分配失败，立方体不可用");
        return false;
    }
    memset(s_buf, 0, kSize * kSize * sizeof(uint16_t));

    DisplayLockGuard lock(s_display);

    // 挂在 lv_layer_top() 上：盖在所有页面（含状态栏）之上，不用改原本的 UI 结构
    s_root = lv_obj_create(lv_layer_top());
    lv_obj_set_size(s_root, kSize, kSize);
    lv_obj_center(s_root);
    lv_obj_remove_flag(s_root, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(s_root, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_root, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_root, 0, 0);
    lv_obj_set_style_pad_all(s_root, 0, 0);
    lv_obj_set_style_radius(s_root, 0, 0);

    s_canvas = lv_canvas_create(s_root);
    lv_canvas_set_buffer(s_canvas, s_buf, kSize, kSize, LV_COLOR_FORMAT_RGB565);
    lv_obj_center(s_canvas);

    lv_obj_add_flag(s_root, LV_OBJ_FLAG_HIDDEN);  // 默认不显示

    // 优先级 2：低于音频任务，画面卡一点也不能断音频
    if (xTaskCreate(RenderTask, "cube_render", 4096, nullptr, 2, &s_task) != pdPASS) {
        ESP_LOGE(TAG, "渲染任务创建失败");
        return false;
    }

    ESP_LOGI(TAG, "立方体就绪（%dx%d，缓冲 %d KB）", kSize, kSize,
             (int)(kSize * kSize * 2 / 1024));
    return true;
}

void Show() {
    if (s_root == nullptr || s_buf == nullptr) {
        ESP_LOGW(TAG, "还没初始化");
        return;
    }
    if (s_visible) {
        return;
    }
    s_visible = true;

    DisplayLockGuard lock(s_display);
    lv_obj_remove_flag(s_root, LV_OBJ_FLAG_HIDDEN);
    ESP_LOGI(TAG, "显示立方体");
}

void Hide() {
    if (!s_visible) {
        return;
    }
    s_visible = false;
    DisplayLockGuard lock(s_display);
    lv_obj_add_flag(s_root, LV_OBJ_FLAG_HIDDEN);
    ESP_LOGI(TAG, "隐藏立方体");
}

bool IsVisible() {
    return s_visible;
}

void SetAttitude(float pitch, float roll, float yaw) {
    s_pitch = pitch;
    s_roll = roll;
    s_yaw = yaw;
}

}  // namespace cubetoy
