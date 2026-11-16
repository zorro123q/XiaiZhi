#ifndef VISUAL_FOLLOW_H
#define VISUAL_FOLLOW_H

#include <functional>
#include <string>

/**
 * 色块跟随：让小车用摄像头盯住一个高饱和度的物体并保持距离。
 *
 * 为什么这条路走得通 —— 本板摄像头在 esp_sparkbot_board.cc 里配的是
 * PIXFORMAT_RGB565 / FRAMESIZE_240X240，拿到的是原始像素而不是 JPEG，
 * 所以可以逐像素直接判色，不需要解码。而 ESP32-S3 扫一遍 240x240 大约
 * 4~6ms（含 PSRAM 读取），15fps 只吃掉不到 10% 的 CPU。
 *
 * 为什么不需要编码器 —— 底盘没有编码器、串口单向只发不收，方向本来只能
 * 开环估。但跟随这件事的角度反馈直接来自图像：目标偏左就往左打，打过了
 * 就往回修。相机在这里同时充当了传感器和位置反馈。
 *
 * 边界：只认颜色，不认"人""杯子"这种语义。要语义跟随得引入 esp-dl 和
 * 量化模型，那条路要改分区表、帧率也会掉到 2~3fps。
 */
namespace vfollow {

// 跟随目标的颜色。RGB565 里绿色是 6 位、红蓝是 5 位，量化步长不同，
// 所以三个色相的通道差阈值不能照抄同一套。
enum class TargetHue {
    kRed = 0,
    kGreen = 1,
    kBlue = 2,
};

// 把归一化的控制量交给调用方：x 是转向（正=右转），y 是速度（正=前进），范围 [-1,1]。
using MotionSink = std::function<void(float x, float y)>;
// 任务自己结束（跑满时限、长时间找不到目标）时的收尾回调，调用方负责真正停车。
using ExitSink = std::function<void()>;

struct Config {
    TargetHue hue = TargetHue::kRed;
    // 希望目标占据的画面比例。比它小就前进、比它大就后退，所以这是"跟多远"的
    // 旋钮而不是速度旋钮。调大 = 跟得更近。
    float target_area = 0.08f;
    float kp = 1.2f;
    // 转向微分项。底盘没有编码器，车身的左右摆动全靠这一项压住。
    float kd = 0.35f;
    float max_speed = 0.55f;
    int max_duration_ms = 180000;
};

// 起一个独立的跟随任务。它自己会持续驱动底盘，调用方不要在工具回调里等它。
// 返回 false 表示上一轮还没收尾完，或者任务创建失败。
bool Start(const Config& cfg, MotionSink on_motion, ExitSink on_exit);

// 只置停止标志后立即返回，不等待任务退出。
// 这个函数会在 MCP 工具回调里被调用，而工具回调跑在 Application 主任务上，
// 阻塞几百毫秒就会冻住音频和显示。
void Stop();

bool IsRunning();

// 最近一帧的识别结果，JSON 字符串。看不到串口时用它诊断"为什么车不动"。
// 注意全程整数输出：sdkconfig 开了 CONFIG_NEWLIB_NANO_FORMAT，printf 不支持 %f。
std::string GetStatusJson();

}  // namespace vfollow

#endif  // VISUAL_FOLLOW_H
