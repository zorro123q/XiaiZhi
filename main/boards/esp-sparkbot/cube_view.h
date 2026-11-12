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
 * 第 1 轮：角度由固定速度自转驱动。
 * 第 2 轮：接上 IMU 后改调 SetAttitude()，由板子姿态驱动。
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

// 供第 2 轮 IMU 使用：单位为度
void SetAttitude(float pitch, float roll, float yaw);

}  // namespace cubetoy

#endif  // CUBE_VIEW_H
