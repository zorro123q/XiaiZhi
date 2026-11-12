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
    // 跳舞前用户选的模式，舞毕恢复。原来固定切休眠会把用户设置吞掉
    light_mode_t pre_dance_light_mode_ = LIGHT_MODE_ALWAYS_ON;

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

    // ── 原地转向标定 ────────────────────────────────────────────────
    // 整条链路没有任何角度反馈：底盘没有编码器、串口是单向只发不收、
    // 主控板上也没有 IMU。所以"转一圈"只能靠"转速 × 时间"去逼近。
    // 转速还随电池电压漂，这是个开环估计，不是闭环测量。
    // turn_deg_per_sec_ 是唯一能把角度换算成时长的参数，必须实测校准：
    //   让车满速原地转一整圈，记下用了几秒 → deg_per_sec = 360 / 秒数。
    // 出厂默认 90°/s（4 秒一圈）纯属估值，实测后务必用
    // self.chassis.set_turn_calibration 覆盖，值会写进 NVS。
    int turn_deg_per_sec_ = 90;
    static constexpr int kTurnDpsMin = 20;          // 18 秒转一圈（很慢的底盘）
    static constexpr int kTurnDpsMax = 360;         // 1 秒转一圈（很暴力的底盘）
    static constexpr int kTurnDurationMaxMs = 30000;  // 单次转向时长上限，防呆
    static constexpr int kTurnDeadbandMs = 100;     // 少于这么多毫秒不值得动电机

    // 跳舞灯光联动：底盘是单向串口（只发不收），跳舞何时结束拿不到回执，
    // 只能按固件里写死的动作表时长开一次性定时器，到点由 OnDanceLightTimeout 切休眠。
    //
    // 命令顺序必须是 w7 → d1，不能反过来：
    //   c2_tracked_chassis 的 echo_task 收到 'd' 后同步调用 spark_bot_dance()，
    //   内部 vTaskDelay 逐段跑完整套动作（合计 kDanceDurationMs），期间完全不执行
    //   uart_read_bytes。此时再发的 w7 只会躺在 RX 缓冲里，等舞蹈结束才被读到——
    //   实测表现就是"跳完了才亮灯"。
    //
    // 两条命令还必须错峰：底盘用 sscanf("%c%f") 只解析第一组命令，
    // 同毫秒发出会在总线上连成 "w7d1"，第二条被静默丢弃。
    esp_timer_handle_t dance_cmd_timer_ = nullptr;    // w7 → d1 的错峰延时
    esp_timer_handle_t dance_light_timer_ = nullptr;  // 舞蹈结束 → w8
    // 底盘舞蹈固定时长：motion_control.c 的 dance_moves 表逐项求和，两段各 7900ms
    static constexpr int kDanceDurationMs = 15800;
    // 向上取整到秒（15800 → 16s），多出的 200ms 正好当舞蹈结束后的缓冲
    static constexpr int kDanceLightDefaultSec = (kDanceDurationMs + 999) / 1000;
    static constexpr int kDanceCmdGapMs = 150;        // 错峰时长，够底盘吃完 w7

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

#if CONFIG_USE_SERVER_AEC
        // 实时对话（服务器端 AEC）开关。本板只有一颗 ES8311，没有回声参考通道，
        // 所以用不了设备端 AEC（CONFIG_USE_DEVICE_AEC），只能让服务器拿它自己下发的
        // TTS 做参考来消回声。开启后设备播放回复时保持拾音，可以直接说话打断。
        // 用长按而不是双击：注册双击会让上面单击的响应多等一个双击窗口，伤手感。
        boot_button_.OnLongPress([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() != kDeviceStateIdle) {
                return;
            }
            bool turn_on = app.GetAecMode() == kAecOff;
            app.SetAecMode(turn_on ? kAecOnServerSide : kAecOff);
            Settings settings("sparkbot", true);
            settings.SetBool("realtime", turn_on);
            ESP_LOGI(TAG, "Realtime conversation %s (saved)", turn_on ? "enabled" : "disabled");
        });

        // 开机恢复上次的选择。默认构造出来就是开启态，所以只有"关闭"需要动作，
        // 正常开启时不去碰它，免得开机白弹一条通知。
        {
            Settings settings("sparkbot", false);
            bool realtime = settings.GetBool("realtime", true);
            if (!realtime) {
                Application::GetInstance().SetAecMode(kAecOff);
            }
            ESP_LOGI(TAG, "Realtime conversation (server AEC) %s, long-press BOOT to toggle",
                     realtime ? "enabled" : "disabled");
        }
#endif
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

        // 开机默认常亮：指令由 light_mode_ 派生，避免和成员初值改动后不同步
        char init_light_cmd[3] = {
            'w', static_cast<char>(static_cast<int>(light_mode_) + '0'), 0};
        SendUartMessage(init_light_cmd);

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

        // w7 → d1 的错峰定时器，避免两条命令在串口上粘成一条
        esp_timer_create_args_t cmd_timer_args = {
            .callback = [](void* arg) {
                static_cast<EspSparkBot*>(arg)->OnDanceCmdTick();
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "dance_cmd",
            .skip_unhandled_events = true,
        };
        ESP_ERROR_CHECK(esp_timer_create(&cmd_timer_args, &dance_cmd_timer_));

        // 巡航速度从 NVS 恢复（set_speed 工具会写进去）
        Settings settings("sparkbot", false);
        int stored_speed = settings.GetInt("cruise-speed", 100);
        cruise_speed_ = (stored_speed >= 10 && stored_speed <= 100) ? stored_speed : 100;

        // 转向标定从 NVS 恢复（set_turn_calibration 工具会写进去）
        int stored_dps = settings.GetInt("turn-deg-per-sec", 90);
        turn_deg_per_sec_ = (stored_dps >= kTurnDpsMin && stored_dps <= kTurnDpsMax) ? stored_dps : 90;
        ESP_LOGI(TAG, "转向标定：%d 度/秒（一圈约 %d ms）",
                 turn_deg_per_sec_, 360000 / turn_deg_per_sec_);

        // 运动重发任务：负责把"走 N 秒"翻译成持续的 x/y 指令
        motion_mutex_ = xSemaphoreCreateMutex();
        xTaskCreate(MotionTaskEntry, "chassis_motion", kMotionStackSize, this, 5, nullptr);
    }

    // 编号 → 中文名。给模型看的返回值必须自带名字：
    // 只回一个裸数字 2，模型会自己脑补成"第 2 种"再换算成 w4，就是之前错位的根源。
    static const char* LightModeName(light_mode_t mode) {
        switch (mode) {
            case LIGHT_MODE_CHARGING_BREATH:   return "充电呼吸";
            case LIGHT_MODE_POWER_LOW:         return "低电提示";
            case LIGHT_MODE_ALWAYS_ON:         return "常亮";
            case LIGHT_MODE_BLINK:             return "闪烁";
            case LIGHT_MODE_WHITE_BREATH_SLOW: return "白色慢呼吸";
            case LIGHT_MODE_WHITE_BREATH_FAST: return "白色快呼吸";
            case LIGHT_MODE_FLOWING:           return "流水";
            case LIGHT_MODE_SHOW:              return "灯光秀";
            case LIGHT_MODE_SLEEP:             return "休眠";
            default:                           return "未知";
        }
    }

    void SendUartMessage(const char * command_str) {
        uint8_t len = strlen(command_str);
        uart_write_bytes(ECHO_UART_PORT_NUM, command_str, len);
        ESP_LOGI(TAG, "Sent command: %s", command_str);
    }

    // 跳舞开始：立刻点亮灯秀(w7)，错峰后再发跳舞指令(d1)，舞毕恢复跳舞前的模式
    void StartDanceSequence(int total_sec) {
        if (dance_light_timer_ == nullptr || dance_cmd_timer_ == nullptr) {
            ESP_LOGW(TAG, "跳舞灯光定时器未就绪，本次不做联动");
            SendUartMessage("d1");
            return;
        }
        // 记下用户跳舞前的选择，舞毕恢复；必须写在下面覆盖 light_mode_ 之前
        pre_dance_light_mode_ = light_mode_;
        // 1) 先点灯，再跳舞。反过来的话 d1 会阻塞底盘 15.8 秒不读串口，
        //    w7 得等舞蹈结束才被处理（实测表现：跳完才亮）
        SendUartMessage("w7");
        light_mode_ = LIGHT_MODE_SHOW;
        // 2) 错峰 150ms 再发 d1，避免两条命令粘成 "w7d1" 被 sscanf 只吃掉第一条
        esp_timer_stop(dance_cmd_timer_);
        esp_timer_start_once(dance_cmd_timer_, static_cast<uint64_t>(kDanceCmdGapMs) * 1000ULL);
        // 3) 舞蹈跑完再恢复跳舞前的模式
        esp_timer_stop(dance_light_timer_);
        esp_timer_start_once(dance_light_timer_,
                             static_cast<uint64_t>(total_sec) * 1000000ULL);
        ESP_LOGI(TAG, "跳舞灯光：已切灯秀，%dms 后发 d1，%d 秒后恢复到 %d(%s)",
                 kDanceCmdGapMs, total_sec,
                 static_cast<int>(pre_dance_light_mode_),
                 LightModeName(pre_dance_light_mode_));
    }

    // 错峰到点，此时 w7 已被底盘读走，d1 单独成帧
    void OnDanceCmdTick() {
        SendUartMessage("d1");
    }

    // 用户手动切灯时调用：取消"舞毕恢复"的预定动作，别把用户的新选择覆盖掉。
    // 注意不动 dance_cmd_timer_：切灯不该打断已经在进行的跳舞。
    void CancelDanceLight() {
        if (dance_light_timer_ == nullptr) {
            return;
        }
        // 用返回值判断是否真的取消了（未运行/已触发完成时返回错误码，幂等）
        if (esp_timer_stop(dance_light_timer_) == ESP_OK) {
            ESP_LOGI(TAG, "跳舞灯光联动已取消");
        }
    }

    // esp_timer 任务上下文：只发串口 + 改状态，不做阻塞操作
    void OnDanceLightTimeout() {
        // 恢复跳舞前的模式，而不是写死 w8。
        // 旧实现无条件切休眠，用户跳完就从"常亮/慢呼吸"掉到近乎全灭，
        // 体感像"逻辑断了"。
        light_mode_t restore = pre_dance_light_mode_;
        char command_str[3] = {'w', static_cast<char>(static_cast<int>(restore) + '0'), 0};
        SendUartMessage(command_str);
        light_mode_ = restore;
        ESP_LOGI(TAG, "跳舞结束，灯光恢复到 %d(%s)",
                 static_cast<int>(restore), LightModeName(restore));
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
        // 时长必须打出来：模型给的 duration 是它自己猜的，"转不够/走太远"的时候
        // 只有这个数能证明到底是它给少了，还是车本身跑得慢。
        ESP_LOGI(TAG, "底盘运动 x=%d/100 y=%d/100 %s %dms", static_cast<int>(x * 100),
                 static_cast<int>(y * 100), continuous ? "持续直到停车" : "定时",
                 continuous ? kContinuousMaxMs : duration_ms);
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

    // 角度 → 时长 → 原地转。degrees 正数右转、负数左转。返回本次实际时长（ms）。
    // 注意这是开环估计：标定值准不准、电池电压高不高，都会影响实际角度。
    int StartTurnByDegrees(int degrees) {
        int magnitude = degrees >= 0 ? degrees : -degrees;
        int ms = static_cast<int>(static_cast<long long>(magnitude) * 1000 / turn_deg_per_sec_);
        if (ms > kTurnDurationMaxMs) {
            ms = kTurnDurationMaxMs;
        }
        if (ms < kTurnDeadbandMs) {
            ms = kTurnDeadbandMs;
        }
        StartMotion(degrees > 0 ? 1.0f : -1.0f, 0.0f, ms);
        ESP_LOGI(TAG, "原地转 %d 度 → %dms（标定 %d 度/秒）", degrees, ms, turn_deg_per_sec_);
        return ms;
    }

    // 校准：把实测的"满速原地转一圈用了几秒"换算成度/秒并持久化。
    // 这是让"转一圈"真的变成一圈的唯一入口，不要把标定值硬编码在代码里。
    int SetTurnCalibration(int seconds_per_rev) {
        if (seconds_per_rev < 1) {
            seconds_per_rev = 1;
        } else if (seconds_per_rev > 18) {
            seconds_per_rev = 18;
        }
        int dps = 360 / seconds_per_rev;
        if (dps < kTurnDpsMin) {
            dps = kTurnDpsMin;
        } else if (dps > kTurnDpsMax) {
            dps = kTurnDpsMax;
        }
        turn_deg_per_sec_ = dps;

        Settings settings("sparkbot", true);
        settings.SetInt("turn-deg-per-sec", dps);
        ESP_LOGI(TAG, "转向标定：%d 秒/圈 → %d 度/秒", seconds_per_rev, dps);
        return dps;
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
        // 返回值带上中文名，别只回裸整数：模型拿到 2 会当"第 2 种"再 +2 换算出 w4。
        // 编号就是底盘协议号，读写同一套，不做任何序号换算。
        mcp_server.AddTool("self.chassis.get_light_mode", "获取当前灯光模式，返回“编号(名称)”，例如 2(常亮)。这个编号和 switch_light_mode 的参数是同一套，直接原样转述给用户", PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
            char buf[48];
            snprintf(buf, sizeof(buf), "%d(%s)",
                     static_cast<int>(light_mode_), LightModeName(light_mode_));
            return std::string(buf);
        });

        // 说明：底盘只认"速度"，不认"距离"。持续时长由头部负责计时与主动停车，
        // 参数 duration 让模型能按"走多远"自行选择秒数（1 秒约 30~40 厘米）。
        PropertyList motion_args = PropertyList({
            Property("duration", kPropertyTypeInteger, 2, 1, 10)
        });

        mcp_server.AddTool("self.chassis.move", "让小车一直按指定速度走，直到调用停车工具或 60 秒上限。speed 正数前进、负数后退；turn 正数右转、负数左转；都是 -100~100 的百分比。用户说“一直往前走”“继续走”“开快点”时用这个。用户给了具体角度（转多少度、转一圈）时必须用 turn_degrees，不要拿 turn 参数去凑角度", PropertyList({
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

        // 兜底的"随便转一下"。用户给了具体角度时应该走 turn_degrees，
        // 那里才有时长按角度换算；这里只能靠 duration 猜，所以描述里明确分流。
        mcp_server.AddTool("self.chassis.turn_left", "原地左转一小段（转完自动停）。duration 为持续秒数（1-10），越大转得越多。仅当用户没说角度、只想随便转一下时用；用户说了角度或“一圈”“半圈”请用 turn_degrees", motion_args, [this](const PropertyList& properties) -> ReturnValue {
            StartMotion(-1.0f, 0.0f, properties["duration"].value<int>() * 1000);
            return true;
        });

        mcp_server.AddTool("self.chassis.turn_right", "原地右转一小段（转完自动停）。duration 为持续秒数（1-10），越大转得越多。仅当用户没说角度、只想随便转一下时用；用户说了角度或“一圈”“半圈”请用 turn_degrees", motion_args, [this](const PropertyList& properties) -> ReturnValue {
            StartMotion(1.0f, 0.0f, properties["duration"].value<int>() * 1000);
            return true;
        });

        // 角度入口。整个系统没有角度反馈（无编码器/无 IMU/串口单向），
        // 实际角度 = 标定转速 × 时长，是个开环估计，会随电池电压漂。
        //
        // ⚠️ 这个工具是"立即返回"的：MCP 回调跑在 Application 主任务里
        // （日志 "Send MCP message in main thread" 可证），绝不能在这里阻塞等车转完，
        // 否则音频、显示、状态机会一起冻结。所以模型拿到响应时车还在转。
        // 实测踩过的坑：说"转一圈"，模型 0.6 秒就播"转完一圈啦"，而车还要转 3.4 秒。
        // 对策是把"还没转完"写进返回值里，用返回值约束模型的下一句话。
        mcp_server.AddTool("self.chassis.turn_degrees",
            "原地转动指定角度（转完自动停）。degrees 是角度数值本身，用户说多少就传多少，不做任何换算："
            "“转一圈”传 360，“转半圈”“掉头”传 180，“转四分之一圈”“转 90 度”传 90，“转两圈”传 720；"
            "正数右转、负数左转。用户只要提到具体角度，或“一圈”“半圈”“掉头”这类量词，一律用这个工具，不要用 turn_left/turn_right。"
            "注意本工具立即返回，车会在后台继续转，返回值里写的持续时间才是它真正停下的时刻",
            PropertyList({
            Property("degrees", kPropertyTypeInteger, 90, -1080, 1080)
        }), [this](const PropertyList& properties) -> ReturnValue {
            int degrees = properties["degrees"].value<int>();
            if (degrees == 0) {
                StopMotion();
                return std::string("没有转动，车本来就是停的");
            }
            int ms = StartTurnByDegrees(degrees);
            // UTF-8 中文一个字符 3 字节，这句提示约 320 字节，缓冲必须留够，
            // 否则会在多字节中间截断成乱码
            char buf[512];
            snprintf(buf, sizeof(buf),
                     "已启动原地%s %d 度，车会转 %d 毫秒（约 %d 秒）后自动停。"
                     "注意：现在是刚开始转，还没转完。回复用户时不要说“转完了”“已经转好”，"
                     "只能说“正在转”“大约 %d 秒后停下”；要确认结果请稍后调用 get_status",
                     degrees > 0 ? "右转" : "左转", degrees, ms, (ms + 500) / 1000,
                     (ms + 500) / 1000);
            return std::string(buf);
        });

        // 校准入口。没有它，turn_degrees 里的标定值就只能靠代码硬编码改固件。
        mcp_server.AddTool("self.chassis.set_turn_calibration",
            "校准原地转向速度。seconds_per_rev = 让车满速原地转一整圈实际用了几秒（1-18 的整数，取最接近的整秒）。"
            "只有在用户配合实测（看着车转完一圈并计时），或者用户反馈“转的角度不准”时才调用；"
            "不要自己估一个值随便调。会存进 NVS，重启后仍然有效",
            PropertyList({
            Property("seconds_per_rev", kPropertyTypeInteger, 4, 1, 18)
        }), [this](const PropertyList& properties) -> ReturnValue {
            int sec = properties["seconds_per_rev"].value<int>();
            int dps = SetTurnCalibration(sec);
            return std::string("已校准：一圈 ") + std::to_string(sec) + " 秒，" +
                   std::to_string(dps) + " 度/秒";
        });

        // 让模型能读回标定值，否则"校准过了没有"它自己都不知道
        mcp_server.AddTool("self.chassis.get_turn_calibration",
            "获取当前的原地转向标定值，返回“度/秒”和推算的“一圈多少毫秒”。用户问“现在转一圈要多久”时用它",
            PropertyList(), [this](const PropertyList& properties) -> ReturnValue {
                return std::string("{\"deg_per_sec\":") + std::to_string(turn_deg_per_sec_) +
                       ",\"ms_per_rev\":" + std::to_string(360000 / turn_deg_per_sec_) + "}";
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

        mcp_server.AddTool("self.chassis.dance", "跳舞。开跳前先点亮灯光秀，舞蹈结束后自动恢复到跳舞前的灯光模式。底盘舞蹈动作表固定约 16 秒，通常不用传 duration", PropertyList({
            Property("duration", kPropertyTypeInteger, kDanceLightDefaultSec, 15, 60)
        }), [this](const PropertyList& properties) -> ReturnValue {
            StopMotion();  // 底盘跳舞期间不读串口，先停掉重发任务避免指令堆积
            StartDanceSequence(properties["duration"].value<int>());
            return true;
        });

        mcp_server.AddTool("self.chassis.switch_light_mode",
            "切换底盘灯光。参数 light_mode 就是底盘协议编号，与 get_light_mode 返回的编号完全一致："
            "用户说“模式 N”“第 N 号”就直接传 N，不做任何加减换算。"
            "0=充电呼吸 1=低电提示 2=常亮 3=闪烁 4=白色慢呼吸 5=白色快呼吸 6=流水 7=灯光秀 8=休眠。"
            "注意 0/1 由电池状态自动驱动，手动切过去会被覆盖，一般不要用",
            PropertyList({
            Property("light_mode", kPropertyTypeInteger, 2, 0, 8)
        }), [this](const PropertyList& properties) -> ReturnValue {
            int mode = properties["light_mode"].value<int>();
            if (mode < 0 || mode >= LIGHT_MODE_MAX) {
                ESP_LOGW(TAG, "非法灯光模式: %d", mode);
                return false;
            }
            char command_str[3] = {'w', static_cast<char>(mode + '0'), 0};

            ESP_LOGI(TAG, "Switch Light Mode: %d(%s)", mode,
                     LightModeName(static_cast<light_mode_t>(mode)));

            SendUartMessage(command_str);
            // 手动切灯优先：取消跳舞结束后“恢复原模式”的预定动作
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
