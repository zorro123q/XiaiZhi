#include "wifi_board.h"
#include "codecs/es8311_audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "settings.h"

#include <wifi_station.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_lcd_panel_vendor.h>
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <cstdio>
#include <cstring>

#include "esp32_camera.h"
#include "wifi_remote_control.h"
#include "sparkbot_battery.h"

#define TAG "esp_sparkbot"

LV_FONT_DECLARE(font_puhui_20_4);
LV_FONT_DECLARE(font_awesome_20_4);

class SparkBotEs8311AudioCodec : public Es8311AudioCodec {
private:    

public:
    SparkBotEs8311AudioCodec(void* i2c_master_handle, i2c_port_t i2c_port, int input_sample_rate, int output_sample_rate,
                        gpio_num_t mclk, gpio_num_t bclk, gpio_num_t ws, gpio_num_t dout, gpio_num_t din,
                        gpio_num_t pa_pin, uint8_t es8311_addr, bool use_mclk = true)
        : Es8311AudioCodec(i2c_master_handle, i2c_port, input_sample_rate, output_sample_rate,
                             mclk,  bclk,  ws,  dout,  din,pa_pin,  es8311_addr,  use_mclk = true) {}

    void EnableOutput(bool enable) override {
        if (enable == output_enabled_) {
            return;
        }
        if (enable) {
            Es8311AudioCodec::EnableOutput(enable);
        } else {
           // Nothing todo because the display io and PA io conflict
        }
    }
};

class EspSparkBot : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Button boot_button_;
    Display* display_;
    Esp32Camera* camera_;
    SparkBotBattery* battery_ = nullptr;
    light_mode_t light_mode_ = LIGHT_MODE_ALWAYS_ON;

    // 底盘运动状态。底盘的串口看门狗约 500ms 收不到指令就自动刹停（见
    // c2_tracked_chassis/main/tracked_chassis.c 的 uart_read_count > 25），
    // 所以"走一段距离"必须由头部按固定间隔持续重发，而不是只发一次。
    SemaphoreHandle_t motion_mutex_ = nullptr;
    float motion_x_ = 0.0f;
    float motion_y_ = 0.0f;
    TickType_t motion_deadline_ = 0;
    bool motion_running_ = false;
    bool motion_continuous_ = false;  // true = 一直走，直到 stop 或撞上安全上限
    int cruise_speed_ = 100;          // 1-100，短动作和"一直走"的默认速度百分比
    static constexpr int kMotionResendMs = 100;   // 看门狗 500ms，100ms 重发留足余量
    static constexpr int kMotionStackSize = 3072;
    static constexpr int kContinuousMaxMs = 60000;  // 无限运动的安全上限：60 秒

    // 跳舞灯光联动：底盘是单向串口（只发不收），跳舞何时结束拿不到回执，
    // 只能按约定时长开一次性定时器，到点由 OnDanceLightTimeout 把灯切回休眠。
    // 期间若用户手动切换灯光，会取消定时器，避免把用户选的模式又覆盖回休眠。
    //
    // w7 必须与 d1 错开发送：底盘协议无分隔符、按字符流解析，两条命令同毫秒
    // 发出会在总线上连成 "d1w7"，底盘只会执行第一条，w7 被静默丢弃。
    esp_timer_handle_t dance_show_timer_ = nullptr;   // d1 → w7 的错峰延时
    esp_timer_handle_t dance_light_timer_ = nullptr;  // 舞蹈结束 → w8
    bool dance_light_pending_ = false;
    static constexpr int kDanceLightDefaultSec = 12;
    static constexpr int kDanceShowGapMs = 150;       // 错峰时长，够底盘吃完 d1

    void InitializeI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_MOSI_GPIO;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_CLK_GPIO;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            app.ToggleChatState();
        });
    }

    void InitializeDisplay() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;

        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_CS_GPIO;
        io_config.dc_gpio_num = DISPLAY_DC_GPIO;
        io_config.spi_mode = 0;
        io_config.pclk_hz = 40 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片
        ESP_LOGD(TAG, "Install LCD driver");

        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
        
        esp_lcd_panel_reset(panel);
        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, true);
        esp_lcd_panel_disp_on_off(panel, true);
        display_ = new SpiLcdDisplay(panel_io, panel,
                                    DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY,
                                    {
                                        .text_font = &font_puhui_20_4,
                                        .icon_font = &font_awesome_20_4,
                                        .emoji_font = font_emoji_64_init(),
                                    });
    }

    void InitializeCamera() {
        camera_config_t camera_config = {};

        camera_config.pin_pwdn = SPARKBOT_CAMERA_PWDN;
        camera_config.pin_reset = SPARKBOT_CAMERA_RESET;
        camera_config.pin_xclk = SPARKBOT_CAMERA_XCLK;
        camera_config.pin_pclk = SPARKBOT_CAMERA_PCLK;
        camera_config.pin_sccb_sda = SPARKBOT_CAMERA_SIOD;
        camera_config.pin_sccb_scl = SPARKBOT_CAMERA_SIOC;

        camera_config.pin_d0 = SPARKBOT_CAMERA_D0;
        camera_config.pin_d1 = SPARKBOT_CAMERA_D1;
        camera_config.pin_d2 = SPARKBOT_CAMERA_D2;
        camera_config.pin_d3 = SPARKBOT_CAMERA_D3;
        camera_config.pin_d4 = SPARKBOT_CAMERA_D4;
        camera_config.pin_d5 = SPARKBOT_CAMERA_D5;
        camera_config.pin_d6 = SPARKBOT_CAMERA_D6;
        camera_config.pin_d7 = SPARKBOT_CAMERA_D7;

        camera_config.pin_vsync = SPARKBOT_CAMERA_VSYNC;
        camera_config.pin_href = SPARKBOT_CAMERA_HSYNC;
        camera_config.pin_pclk = SPARKBOT_CAMERA_PCLK;
        camera_config.xclk_freq_hz = SPARKBOT_CAMERA_XCLK_FREQ;
        camera_config.ledc_timer = SPARKBOT_LEDC_TIMER;
        camera_config.ledc_channel = SPARKBOT_LEDC_CHANNEL;
        camera_config.fb_location = CAMERA_FB_IN_PSRAM;
        
        camera_config.sccb_i2c_port = I2C_NUM_0;
        
        camera_config.pixel_format = PIXFORMAT_RGB565;
        camera_config.frame_size = FRAMESIZE_240X240;
        camera_config.jpeg_quality = 12;
        camera_config.fb_count = 2;
        camera_config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
        
        camera_ = new Esp32Camera(camera_config);

        Settings settings("sparkbot", false);
        // 考虑到部分复刻使用了不可动摄像头的设计，默认启用翻转
        bool camera_flipped = static_cast<bool>(settings.GetInt("camera-flipped", 1));
        camera_->SetHMirror(camera_flipped);
        camera_->SetVFlip(camera_flipped);
    }

    // 电池电量检测：主体 PCB 的电池分压接在 ADC2_CH3(GPIO14)
    void InitializeBattery() {
        battery_ = new SparkBotBattery(BATTERY_ADC_UNIT, BATTERY_ADC_CHANNEL,
                                       BATTERY_UPPER_RESISTOR, BATTERY_LOWER_RESISTOR,
                                       BATTERY_CHARGING_PIN);
        if (!battery_->Initialize()) {
            delete battery_;
            battery_ = nullptr;
            ESP_LOGW(TAG, "电池检测不可用，状态栏不显示电量");
        }
    }

    /*
        ESP-SparkBot 的底座
        https://gitee.com/esp-friends/esp_sparkbot/tree/master/example/tank/c2_tracked_chassis
    */
    void InitializeEchoUart() {
        uart_config_t uart_config = {
            .baud_rate = ECHO_UART_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };
        int intr_alloc_flags = 0;

        ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
        ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, UART_ECHO_TXD, UART_ECHO_RXD, UART_ECHO_RTS, UART_ECHO_CTS));

        SendUartMessage("w2");

        // 跳舞结束的灯光回调：一次性定时器，到点把底盘灯切回休眠
        esp_timer_create_args_t dance_timer_args = {
            .callback = [](void* arg) {
                static_cast<EspSparkBot*>(arg)->OnDanceLightTimeout();
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "dance_light",
            .skip_unhandled_events = true,
        };
        ESP_ERROR_CHECK(esp_timer_create(&dance_timer_args, &dance_light_timer_));

        // d1 → w7 的错峰定时器，避免两条命令在串口上粘成一条
        esp_timer_create_args_t show_timer_args = {
            .callback = [](void* arg) {
                static_cast<EspSparkBot*>(arg)->OnDanceShowTick();
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "dance_show",
            .skip_unhandled_events = true,
        };
        ESP_ERROR_CHECK(esp_timer_create(&show_timer_args, &dance_show_timer_));

        // 巡航速度从 NVS 恢复（set_speed 工具会写进去）
        Settings settings("sparkbot", false);
        int stored_speed = settings.GetInt("cruise-speed", 100);
        cruise_speed_ = (stored_speed >= 10 && stored_speed <= 100) ? stored_speed : 100;

        // 运动重发任务：负责把"走 N 秒"翻译成持续的 x/y 指令
        motion_mutex_ = xSemaphoreCreateMutex();
        xTaskCreate(MotionTaskEntry, "chassis_motion", kMotionStackSize, this, 5, nullptr);
    }

    void SendUartMessage(const char * command_str) {
        uint8_t len = strlen(command_str);
        uart_write_bytes(ECHO_UART_PORT_NUM, command_str, len);
        ESP_LOGI(TAG, "Sent command: %s", command_str);
    }

    // 跳舞开始：错峰点亮灯光秀(w7)，并预定 duration_sec 秒后切休眠(w8)
    void StartDanceLight(int duration_sec) {
        light_mode_ = LIGHT_MODE_SHOW;
        if (dance_light_timer_ == nullptr || dance_show_timer_ == nullptr) {
            ESP_LOGW(TAG, "跳舞灯光定时器未就绪，本次不做联动");
            return;
        }
        // 先排好 w7 的错峰发送，再排 w8 的收尾
        esp_timer_stop(dance_show_timer_);
        esp_timer_start_once(dance_show_timer_, static_cast<uint64_t>(kDanceShowGapMs) * 1000ULL);

        esp_timer_stop(dance_light_timer_);
        esp_timer_start_once(dance_light_timer_,
                             static_cast<uint64_t>(duration_sec) * 1000000ULL);
        dance_light_pending_ = true;
        ESP_LOGI(TAG, "跳舞灯光：%dms 后切灯秀，%d 秒后切休眠", kDanceShowGapMs, duration_sec);
    }

    // 错峰到点，此时 d1 已被底盘读走，w7 单独成帧
    void OnDanceShowTick() {
        SendUartMessage("w7");
    }

    // 用户手动切灯时调用：取消"到点切回休眠"，别把用户的选覆盖掉
    void CancelDanceLight() {
        if (dance_light_timer_ == nullptr) {
            return;
        }
        // w7 还没发出去也一并取消，否则会在用户切灯后把灯抢回灯秀
        if (dance_show_timer_ != nullptr) {
            esp_timer_stop(dance_show_timer_);
        }
        // 用返回值判断是否真的取消了（未运行/已触发完成时返回错误码，幂等）
        if (esp_timer_stop(dance_light_timer_) == ESP_OK) {
            dance_light_pending_ = false;
            ESP_LOGI(TAG, "跳舞灯光联动已取消");
        }
    }

    // esp_timer 任务上下文：只发串口 + 改状态，不做阻塞操作
    void OnDanceLightTimeout() {
        dance_light_pending_ = false;
        SendUartMessage("w8");
        light_mode_ = LIGHT_MODE_SLEEP;
        ESP_LOGI(TAG, "跳舞结束，灯光切回休眠");
    }

    // 周期重发的指令不打日志，否则一串口就刷屏
    void SendUartRaw(const char * command_str) {
        uart_write_bytes(ECHO_UART_PORT_NUM, command_str, strlen(command_str));
    }

    // sdkconfig 开了 CONFIG_NEWLIB_NANO_FORMAT，printf 家族不支持 %f，
    // 所以把 [-1,1] 的系数手工拼成 "0.50" / "-1.00" 这种底盘认识的格式
    static void FormatCoefficient(float value, char* out, size_t out_size) {
        bool negative = value < 0.0f;
        if (negative) {
            value = -value;
        }
        int scaled = static_cast<int>(value * 100.0f + 0.5f);  // 0 ~ 100
        snprintf(out, out_size, "%s%d.%02d", negative ? "-" : "", scaled / 100, scaled % 100);
    }

    // duration_ms > 0：走这么久后主动停车；duration_ms <= 0：一直走，直到 stop
    // （仍然保留 60 秒安全上限，避免模型忘了停车导致小车撞墙）
    void StartMotion(float x, float y, int duration_ms) {
        if (motion_mutex_ == nullptr) {
            return;
        }
        if (xSemaphoreTake(motion_mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGW(TAG, "底盘状态忙，忽略本次运动指令");
            return;
        }
        bool continuous = duration_ms <= 0;
        motion_x_ = x;
        motion_y_ = y;
        motion_continuous_ = continuous;
        motion_deadline_ = xTaskGetTickCount() + pdMS_TO_TICKS(continuous ? kContinuousMaxMs : duration_ms);
        motion_running_ = true;
        xSemaphoreGive(motion_mutex_);
        ESP_LOGI(TAG, "底盘运动 x=%d/100 y=%d/100 %s", static_cast<int>(x * 100),
                 static_cast<int>(y * 100), continuous ? "持续直到停车" : "定时");
    }

    // 调速度：存 NVS，正在走的话同步改当前速度（原地转向时不改，免得突然窜出去）
    void SetCruiseSpeed(int percent) {
        if (percent < 10) {
            percent = 10;
        } else if (percent > 100) {
            percent = 100;
        }
        cruise_speed_ = percent;

        Settings settings("sparkbot", true);
        settings.SetInt("cruise-speed", percent);

        if (motion_mutex_ == nullptr) {
            return;
        }
        if (xSemaphoreTake(motion_mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
            return;
        }
        if (motion_running_ && (motion_y_ > 0.01f || motion_y_ < -0.01f)) {
            motion_y_ = (motion_y_ > 0.0f) ? (percent / 100.0f) : (-percent / 100.0f);
        }
        xSemaphoreGive(motion_mutex_);
        ESP_LOGI(TAG, "巡航速度设为 %d%%", percent);
    }

    std::string GetMotionStatusJson() {
        bool running = false;
        bool continuous = false;
        int turn = 0;
        int speed = 0;
        if (motion_mutex_ != nullptr && xSemaphoreTake(motion_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
            running = motion_running_;
            continuous = motion_continuous_;
            turn = static_cast<int>(motion_x_ * 100);
            speed = static_cast<int>(motion_y_ * 100);
            xSemaphoreGive(motion_mutex_);
        }
        return std::string("{\"moving\":") + (running ? "true" : "false") +
               ",\"continuous\":" + (continuous ? "true" : "false") +
               ",\"turn\":" + std::to_string(turn) +
               ",\"speed\":" + std::to_string(speed) +
               ",\"cruise_speed\":" + std::to_string(cruise_speed_) + "}";
    }

    void StopMotion() {
        if (motion_mutex_ == nullptr) {
            return;
        }
        if (xSemaphoreTake(motion_mutex_, pdMS_TO_TICKS(100)) != pdTRUE) {
            return;
        }
        bool was_running = motion_running_;
        motion_running_ = false;
        motion_continuous_ = false;
        xSemaphoreGive(motion_mutex_);
        // 手机摇杆每 150ms 都会调一次，只在真的从"在走"变"停"时打日志
        if (was_running) {
            ESP_LOGI(TAG, "底盘停止");
        }
    }

    static void MotionTaskEntry(void* arg) {
        static_cast<EspSparkBot*>(arg)->MotionTaskLoop();
    }

    void MotionTaskLoop() {
        bool sending = false;
        float last_x = 0.0f;
        float last_y = 0.0f;

        while (true) {
            vTaskDelay(pdMS_TO_TICKS(kMotionResendMs));

            float x = 0.0f;
            float y = 0.0f;
            bool should_send = false;
            bool expired = false;
            bool hit_safety_cap = false;

            if (xSemaphoreTake(motion_mutex_, pdMS_TO_TICKS(50)) == pdTRUE) {
                if (motion_running_) {
                    // 持续运动也受 60 秒安全上限约束，到点一律停车
                    if (xTaskGetTickCount() < motion_deadline_) {
                        x = motion_x_;
                        y = motion_y_;
                        should_send = true;
                    } else {
                        hit_safety_cap = motion_continuous_;
                        motion_running_ = false;
                        motion_continuous_ = false;
                        expired = true;
                    }
                }
                xSemaphoreGive(motion_mutex_);
            } else if (sending) {
                // 一时拿不到锁就沿用上一拍，别让底盘看门狗误触发停车
                x = last_x;
                y = last_y;
                should_send = true;
            }

            if (should_send) {
                char xs[8];
                char ys[8];
                char buf[24];
                FormatCoefficient(x, xs, sizeof(xs));
                FormatCoefficient(y, ys, sizeof(ys));
                snprintf(buf, sizeof(buf), "x%s y%s", xs, ys);
                SendUartRaw(buf);
                last_x = x;
                last_y = y;
                sending = true;
            } else if (sending) {
                SendUartRaw("x0.00 y0.00");
                sending = false;
                if (hit_safety_cap) {
                    ESP_LOGW(TAG, "持续运动已到 60 秒上限，自动停车");
                } else if (expired) {
                    ESP_LOGI(TAG, "底盘运动结束，已主动停车");
                }
            }
        }
    }

    void InitializeTools() {
        auto& mcp_server = McpServer::GetInstance();
        // 定义设备的属性
        mcp_server.AddTool("self.chassis.get_light_mode", "获取当前灯光效果编号（3=闪烁 4=慢呼吸 5=快呼吸 6=流水 7=灯秀 8=休眠；0-2 为充电/低电/常亮等自动模式）", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
            return static_cast<int>(light_mode_);
        });

        // 说明：底盘只认"速度"，不认"距离"。持续时长由头部负责计时与主动停车，
        // 参数 duration 让模型能按"走多远"自行选择秒数（1 秒约 30~40 厘米）。
        PropertyList motion_args = PropertyList({
            Property("duration", kPropertyTypeInteger, 2, 1, 10)
        });

        mcp_server.AddTool("self.chassis.move", "让小车一直按指定速度走，直到调用停车工具或 60 秒上限。speed 正数前进、负数后退；turn 正数右转、负数左转；都是 -100~100 的百分比。用户说“一直往前走”“继续走”“开快点”时用这个", PropertyList({
            Property("speed", kPropertyTypeInteger, 100, -100, 100),
            Property("turn", kPropertyTypeInteger, 0, -100, 100)
        }), [this](const PropertyList& properties) -> ReturnValue {
            int speed = properties["speed"].value<int>();
            int turn = properties["turn"].value<int>();
            if (speed == 0 && turn == 0) {
                StopMotion();
            } else {
                StartMotion(turn / 100.0f, speed / 100.0f, 0);
            }
            return true;
        });

        mcp_server.AddTool("self.chassis.go_forward", "向前走一小段（走完自动停）。duration 为持续秒数（1-10），越大走得越远。用户只说“走一步”“往前挪一点”时用这个；要一直走请用 move", motion_args, [this](const PropertyList& properties) -> ReturnValue {
            StartMotion(0.0f, cruise_speed_ / 100.0f, properties["duration"].value<int>() * 1000);
            return true;
        });

        mcp_server.AddTool("self.chassis.go_back", "向后退一小段（退完自动停）。duration 为持续秒数（1-10）", motion_args, [this](const PropertyList& properties) -> ReturnValue {
            StartMotion(0.0f, -cruise_speed_ / 100.0f, properties["duration"].value<int>() * 1000);
            return true;
        });

        mcp_server.AddTool("self.chassis.turn_left", "原地左转一小段（转完自动停）。duration 为持续秒数（1-10），越大转得越多", motion_args, [this](const PropertyList& properties) -> ReturnValue {
            StartMotion(-1.0f, 0.0f, properties["duration"].value<int>() * 1000);
            return true;
        });

        mcp_server.AddTool("self.chassis.turn_right", "原地右转一小段（转完自动停）。duration 为持续秒数（1-10），越大转得越多", motion_args, [this](const PropertyList& properties) -> ReturnValue {
            StartMotion(1.0f, 0.0f, properties["duration"].value<int>() * 1000);
            return true;
        });

        mcp_server.AddTool("self.chassis.stop", "立即停止底盘运动", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
            StopMotion();
            return true;
        });

        mcp_server.AddTool("self.chassis.set_speed", "设置底盘速度百分比（10-100），会存下来供以后的移动使用；正在走时立即改速。调之前可以先调 get_status 看当前值", PropertyList({
            Property("percent", kPropertyTypeInteger, 10, 100)
        }), [this](const PropertyList& properties) -> ReturnValue {
            SetCruiseSpeed(properties["percent"].value<int>());
            return true;
        });

        mcp_server.AddTool("self.chassis.get_status", "获取底盘当前状态：是否在动、是否一直走、当前转向/速度百分比、巡航速度", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
            return GetMotionStatusJson();
        });

        mcp_server.AddTool("self.chassis.dance", "跳舞。跳舞期间自动开启灯光秀，跳完自动切回休眠灯。duration 为舞蹈持续秒数（默认 12，用户说“跳久一点”可调大）", PropertyList({
            Property("duration", kPropertyTypeInteger, kDanceLightDefaultSec, 3, 60)
        }), [this](const PropertyList& properties) -> ReturnValue {
            StopMotion();  // 底盘跳舞期间不读串口，先停掉重发任务避免指令堆积
            SendUartMessage("d1");
            StartDanceLight(properties["duration"].value<int>());
            return true;
        });

        mcp_server.AddTool("self.chassis.switch_light_mode", "切换灯光效果。3=闪烁 4=白色慢呼吸 5=白色快呼吸 6=流水 7=灯光秀 8=休眠。用户说“第一种/第 N 种”时，第 1 种对应 3，依次类推", PropertyList({
            Property("light_mode", kPropertyTypeInteger, 3, 8)
        }), [this](const PropertyList& properties) -> ReturnValue {
            int mode = properties["light_mode"].value<int>();
            char command_str[3] = {'w', static_cast<char>(mode + '0'), 0};

            ESP_LOGI(TAG, "Switch Light Mode: %d", mode);

            SendUartMessage(command_str);
            // 手动切灯优先：取消跳舞结束后“切回休眠”的预定动作
            CancelDanceLight();
            light_mode_ = static_cast<light_mode_t>(mode);
            return true;
        });

        mcp_server.AddTool("self.camera.set_camera_flipped", "翻转摄像头图像方向", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
            Settings settings("sparkbot", true);
            // 考虑到部分复刻使用了不可动摄像头的设计，默认启用翻转
            bool flipped = !static_cast<bool>(settings.GetInt("camera-flipped", 1));
            
            camera_->SetHMirror(flipped);
            camera_->SetVFlip(flipped);
            
            settings.SetInt("camera-flipped", flipped ? 1 : 0);
            
            return true;
        });

        mcp_server.AddTool("self.battery.get_level", "获取设备电池电量（百分比）以及是否正在充电", PropertyList(),
            [this](const PropertyList& properties) -> ReturnValue {
                int level = 0;
                bool charging = false;
                bool discharging = false;
                if (!GetBatteryLevel(level, charging, discharging)) {
                    return std::string("无法读取电池电量，电池检测不可用");
                }
                return std::string("{\"level\":") + std::to_string(level) +
                       ",\"charging\":" + (charging ? "true" : "false") + "}";
            });
    }

public:
    EspSparkBot() : boot_button_(BOOT_BUTTON_GPIO) {
        InitializeI2c();
        InitializeSpi();
        InitializeDisplay();
        InitializeButtons();
        InitializeCamera();
        InitializeBattery();
        InitializeEchoUart();
        InitializeTools();
        GetBacklight()->RestoreBrightness();

        // 手机网页遥控：开热点 + HTTP 服务器，摇杆指令通过串口发给底盘
        // 第二个回调提供摄像头取帧，网页上方可显示实时画面
        // 第三个回调提供电量，网页顶部显示百分比
        // 若影响语音功能，注释掉下面这段即可关闭
        WifiRemoteControl::Start(
            [this](const std::string& cmd) {
                // 手机摇杆一接管就取消语音设定的持续运动，否则松手后车又被重发指令带跑
                StopMotion();
                SendUartMessage(cmd.c_str());
            },
            [this](size_t* len) -> uint8_t* {
                return camera_ != nullptr ? camera_->SnapshotJpeg(len, 70) : nullptr;
            },
            [this](int& level, bool& charging) -> bool {
                bool discharging = false;
                return GetBatteryLevel(level, charging, discharging);
            });
    }

    virtual AudioCodec* GetAudioCodec() override {
         static SparkBotEs8311AudioCodec audio_codec(i2c_bus_, I2C_NUM_0, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_PA_PIN, AUDIO_CODEC_ES8311_ADDR);
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }

    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

    virtual Camera* GetCamera() override {
        return camera_;
    }

    // 供状态栏电量图标、设备状态上报（self.get_device_status）和 MCP 工具使用
    virtual bool GetBatteryLevel(int& level, bool& charging, bool& discharging) override {
        if (battery_ == nullptr || !battery_->IsValid()) {
            return false;
        }
        level = battery_->GetLevel();
        charging = battery_->IsCharging();
        discharging = !charging;
        return true;
    }
};

DECLARE_BOARD(EspSparkBot);
