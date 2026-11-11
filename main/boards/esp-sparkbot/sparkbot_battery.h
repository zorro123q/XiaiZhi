#ifndef SPARKBOT_BATTERY_H
#define SPARKBOT_BATTERY_H

#include "config.h"

#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstdint>

#define BATTERY_TAG "SparkBotBattery"

/*
    ESP-SparkBot 电池电量检测

    硬件：主体 PCB 电池电压 -> 100k/100k 分压 -> ADC2_CH3(GPIO14)
    采样：1 秒一次，10 次滑动平均后按单节锂电池放电曲线折算百分比
    说明：
      - ADC2 与 WiFi 共用射频前端，WiFi 运行时读取可能失败。
        这里不对读取结果做强断言，失败时保留上一次的电量，避免出现"假满电"。
      - 主体板未引出充电检测脚，充电状态用电压趋势估算（无法估算时按未充电处理）。
*/
class SparkBotBattery {
public:
    SparkBotBattery(adc_unit_t unit, adc_channel_t channel,
                    float upper_resistor, float lower_resistor,
                    gpio_num_t charging_pin = GPIO_NUM_NC)
        : unit_(unit),
          channel_(channel),
          charging_pin_(charging_pin),
          voltage_ratio_((upper_resistor + lower_resistor) / lower_resistor) {}

    ~SparkBotBattery() {
        if (timer_ != nullptr) {
            esp_timer_stop(timer_);
            esp_timer_delete(timer_);
        }
        if (adc_handle_ != nullptr) {
            adc_oneshot_del_unit(adc_handle_);
        }
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        if (cali_handle_ != nullptr) {
            adc_cali_delete_scheme_curve_fitting(cali_handle_);
        }
#endif
    }

    // 初始化 ADC 并启动周期采样，返回 false 表示电池检测不可用
    bool Initialize() {
        if (charging_pin_ != GPIO_NUM_NC) {
            gpio_config_t io_conf = {};
            io_conf.intr_type = GPIO_INTR_DISABLE;
            io_conf.mode = GPIO_MODE_INPUT;
            io_conf.pin_bit_mask = 1ULL << charging_pin_;
            io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
            io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
            gpio_config(&io_conf);
        }

        adc_oneshot_unit_init_cfg_t unit_cfg = {
            .unit_id = unit_,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &adc_handle_);
        if (err != ESP_OK) {
            ESP_LOGE(BATTERY_TAG, "电池 ADC 初始化失败(%s)，电量显示不可用", esp_err_to_name(err));
            adc_handle_ = nullptr;
            return false;
        }

        adc_oneshot_chan_cfg_t chan_cfg = {
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_12,
        };
        err = adc_oneshot_config_channel(adc_handle_, channel_, &chan_cfg);
        if (err != ESP_OK) {
            ESP_LOGE(BATTERY_TAG, "电池 ADC 通道配置失败(%s)，电量显示不可用", esp_err_to_name(err));
            return false;
        }

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        adc_cali_curve_fitting_config_t cali_cfg = {
            .unit_id = unit_,
            .chan = channel_,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_12,
        };
        if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &cali_handle_) == ESP_OK) {
            calibrated_ = true;
        } else {
            ESP_LOGW(BATTERY_TAG, "ADC 校准不可用，使用线性换算");
            cali_handle_ = nullptr;
        }
#endif

        // 先采一次，开机就有读数
        ReadSample();

        esp_timer_create_args_t timer_args = {
            .callback = [](void* arg) {
                static_cast<SparkBotBattery*>(arg)->ReadSample();
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "sparkbot_battery",
            .skip_unhandled_events = true,
        };
        esp_timer_create(&timer_args, &timer_);
        if (timer_ == nullptr) {
            ESP_LOGW(BATTERY_TAG, "采样定时器创建失败，电量不会刷新");
        } else {
            esp_timer_start_periodic(timer_, 1000000);  // 1 秒
        }

        ESP_LOGI(BATTERY_TAG, "电池检测已启动：分压比 %.2f，当前 %d mV / %d%%",
                 voltage_ratio_, last_mv_, level_);
        return true;
    }

    uint8_t GetLevel() const { return level_; }
    bool IsCharging() const { return charging_; }
    // 最近一次采样是否有效
    bool IsValid() const { return valid_; }

private:
    static constexpr int kWindowSize = 10;   // 10 次采样做平均（约 10 秒）
    static constexpr int kFailLimit = 5;     // 连续失败次数上限
    static constexpr int kTrendSize = 6;     // 充电趋势窗口：6 × 10 秒 = 60 秒

    // 单节锂电池放电曲线（静置电压 -> 电量）
    struct CurvePoint {
        uint16_t millivolt;
        uint8_t level;
    };
    static constexpr CurvePoint kCurve[] = {
        {3300, 0}, {3500, 5}, {3600, 12}, {3700, 30}, {3750, 45}, {3800, 55},
        {3850, 65}, {3900, 72}, {3950, 80}, {4000, 86}, {4050, 91}, {4100, 95},
        {4200, 100},
    };
    static constexpr int kCurveSize = sizeof(kCurve) / sizeof(kCurve[0]);

    adc_unit_t unit_;
    adc_channel_t channel_;
    gpio_num_t charging_pin_;
    float voltage_ratio_;

    adc_oneshot_unit_handle_t adc_handle_ = nullptr;
    adc_cali_handle_t cali_handle_ = nullptr;
    bool calibrated_ = false;
    esp_timer_handle_t timer_ = nullptr;

    uint16_t samples_mv_[kWindowSize] = {};
    int sample_index_ = 0;
    int sample_count_ = 0;
    int fail_count_ = 0;

    uint16_t trend_mv_[kTrendSize] = {};
    int trend_index_ = 0;
    int trend_count_ = 0;
    int trend_tick_ = 0;

    int last_mv_ = 0;
    uint8_t level_ = 0;
    bool charging_ = false;
    bool valid_ = false;
    int log_counter_ = 0;

    void ReadSample() {
        int raw = 0;
        esp_err_t err = ESP_FAIL;
        // ADC2 与 WiFi 共用模拟前端，偶发读取失败，这里重试几次
        for (int attempt = 0; attempt < 3; attempt++) {
            err = adc_oneshot_read(adc_handle_, channel_, &raw);
            if (err == ESP_OK) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(2));
        }
        if (err != ESP_OK) {
            fail_count_++;
            if (fail_count_ == 1) {
                ESP_LOGW(BATTERY_TAG, "读取电池电压失败(%s)；ESP32-S3 的 ADC2 与 WiFi 共用，"
                              "WiFi 运行时可能读不到", esp_err_to_name(err));
            } else if (fail_count_ == kFailLimit) {
                // 持续失败：保留最后一次有效电量，不产生虚假读数
                ESP_LOGE(BATTERY_TAG, "连续 %d 次读不到电池电压，电量停留在最后一次有效值 %d mV / %d%%",
                         fail_count_, last_mv_, level_);
            }
            return;
        }
        fail_count_ = 0;

        int adc_mv = 0;
        bool ok = calibrated_ && cali_handle_ != nullptr &&
                  adc_cali_raw_to_voltage(cali_handle_, raw, &adc_mv) == ESP_OK;
        if (!ok) {
            // 未烧录校准数据时的线性换算（12bit, 12dB 衰减，满量程约 3100mV）
            adc_mv = raw * 3100 / 4095;
        }
        int battery_mv = static_cast<int>(adc_mv * voltage_ratio_ + 0.5f);

        samples_mv_[sample_index_] = battery_mv;
        sample_index_ = (sample_index_ + 1) % kWindowSize;
        if (sample_count_ < kWindowSize) {
            sample_count_++;
        }

        int sum = 0;
        for (int i = 0; i < sample_count_; i++) {
            sum += samples_mv_[i];
        }
        last_mv_ = sum / sample_count_;
        valid_ = true;

        UpdateLevel(last_mv_);
        UpdateChargingState(last_mv_);

        // 每 30 秒打印一次，方便现场排查
        if (++log_counter_ % 30 == 0) {
            ESP_LOGI(BATTERY_TAG, "电池 %d mV / %d%% / %s", last_mv_, level_,
                     charging_ ? "充电中" : "放电中");
        }
    }

    void UpdateLevel(int millivolt) {
        if (millivolt <= kCurve[0].millivolt) {
            level_ = kCurve[0].level;
            return;
        }
        if (millivolt >= kCurve[kCurveSize - 1].millivolt) {
            level_ = kCurve[kCurveSize - 1].level;
            return;
        }
        for (int i = 1; i < kCurveSize; i++) {
            if (millivolt < kCurve[i].millivolt) {
                int mv0 = kCurve[i - 1].millivolt;
                int mv1 = kCurve[i].millivolt;
                int lv0 = kCurve[i - 1].level;
                int lv1 = kCurve[i].level;
                level_ = lv0 + (millivolt - mv0) * (lv1 - lv0) / (mv1 - mv0);
                return;
            }
        }
    }

    // 没有充电检测脚：比较 60 秒前后的电压，明显上升且接近满电才认为在充电
    void UpdateChargingState(int millivolt) {
        if (charging_pin_ != GPIO_NUM_NC) {
            charging_ = gpio_get_level(charging_pin_) == 0;
            return;
        }

        if (++trend_tick_ < 10) {
            return;
        }
        trend_tick_ = 0;

        trend_mv_[trend_index_] = static_cast<uint16_t>(millivolt);
        trend_index_ = (trend_index_ + 1) % kTrendSize;
        if (trend_count_ < kTrendSize) {
            trend_count_++;
            return;
        }

        // 取窗口里最旧的一次采样做对比
        int oldest = trend_mv_[trend_index_];
        charging_ = (millivolt > 4020) && (millivolt - oldest >= 20);
    }
};

#endif  // SPARKBOT_BATTERY_H
