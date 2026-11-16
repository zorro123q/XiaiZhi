#ifndef CUBE_VIEW_H
#define CUBE_VIEW_H

#include <driver/i2c_master.h>
#include <string>

class Display;

/**
 * 屏幕上的 3D 骰子玩具。
 *
 * 渲染路径是自己写的软光栅（不走 tinyGL，也不用 LVGL 的自定义控件），
 * 所以只依赖 lv_canvas + 一个 RGB565 缓冲区，跨 LVGL 版本没有兼容问题。
 *
 * 交互沿用官方 factory_demo 的手感：先翻滚、再减速、最后吸附到整面。
 * 区别在"结果怎么来"——官方靠 IMU 差分角自然停到某面，本实现先随机定落点再倒推动画，
 * 这样掷的瞬间就能拿到点数，不用等画面停（工具回调里绝不能阻塞）。
 *
 * 第 1 轮：固定自转（已移除）。
 * 第 2 轮：Roll() 掷骰 + 惯性吸附落点。
 * 第 3 轮：接上 IMU 后改调 SetAttitude()，由板子姿态驱动。
 */
namespace cubetoy {

// 上电扫一遍 I2C 总线并读各设备的 0x00 寄存器（芯片 ID），只打日志+存结果。
void ProbeImu(i2c_master_bus_handle_t bus, Display* display);

// 取得扫描结果的人类可读描述（给 MCP 工具用，不必看串口也能知道芯片）
std::string GetProbeSummary();

// 创建对象（挂在 lv_layer_top 上，默认隐藏）。display 用来加 LVGL 锁。
bool Init(Display* display);

void Show();
void Hide();
bool IsVisible();

// 掷一次骰子：随机挑一个"某一面正对镜头"的落点，播放约 1.6 秒的翻滚动画，
// 立即返回最终点数 1~6（失败返回 0）。动画在渲染任务里跑，调用方不阻塞。
// 骰子没显示时会自动先显示出来。
int Roll();

// 供第 3 轮 IMU 使用：单位为度
void SetAttitude(float pitch, float roll, float yaw);

}  // namespace cubetoy

#endif  // CUBE_VIEW_H
