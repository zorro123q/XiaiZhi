#ifndef _BOARD_CONFIG_H_
#define _BOARD_CONFIG_H_

#include <driver/gpio.h>
#include <driver/uart.h>
#include <hal/adc_types.h>

#define AUDIO_INPUT_SAMPLE_RATE  16000
#define AUDIO_OUTPUT_SAMPLE_RATE 16000

#define AUDIO_I2S_GPIO_MCLK GPIO_NUM_45
#define AUDIO_I2S_GPIO_WS GPIO_NUM_41
#define AUDIO_I2S_GPIO_BCLK GPIO_NUM_39
#define AUDIO_I2S_GPIO_DIN  GPIO_NUM_40
#define AUDIO_I2S_GPIO_DOUT GPIO_NUM_42

#define AUDIO_CODEC_PA_PIN       GPIO_NUM_46
#define AUDIO_CODEC_I2C_SDA_PIN  GPIO_NUM_4
#define AUDIO_CODEC_I2C_SCL_PIN  GPIO_NUM_5
#define AUDIO_CODEC_ES8311_ADDR  ES8311_CODEC_DEFAULT_ADDR

#define BUILTIN_LED_GPIO        GPIO_NUM_NC
#define BOOT_BUTTON_GPIO        GPIO_NUM_0
#define VOLUME_UP_BUTTON_GPIO   GPIO_NUM_NC
#define VOLUME_DOWN_BUTTON_GPIO GPIO_NUM_NC

/*
    触摸键（双击打断）
    机头的触摸片是 FPC 贴在结构件上的，被摄像头挤成了左右两块、串接在同一个引脚上；
    左右两侧面还各有一个。官方 BSP 用 TOUCH_PAD_NUM1/2/3 三路，
    ESP32-S3 上分别对应 GPIO1/2/3，本固件里这三个脚全部空闲
    （I2S 占 39-42、I2C 占 4/5、SPI 占 21/43/44/47、UART 占 38/48、PA 占 46、
    ADC 占 14、摄像头占 6-18），所以不冲突。

    ⚠️ 哪一路是"顶部"没有文档写明，只能靠串口日志实测：
    双击顶部，看打印出来的 Touch[n] 是几，然后把 TOUCH_PRIMARY_INDEX 改成那个 n。
    默认先按官方 README 的说法猜 0（NUM1/GPIO1 = 顶部确认键）。
*/
#define TOUCH_SENS_TOP         0.04F   // 顶部两块 FPC 并联、面积大，灵敏度压低防误触
#define TOUCH_SENS_SIDE        0.1F    // 左右侧面，沿用官方 BSP 取值
#define TOUCH_PRIMARY_INDEX    0       // 0=NUM1/GPIO1, 1=NUM2/GPIO2, 2=NUM3/GPIO3

// 两次"按下"的最大间隔，超过就当成两次独立的按下。
// 350 对多数人偏紧（第二次按下常在 300~450ms），放宽到 500 提高成功率。
#define TOUCH_DOUBLE_CLICK_MS   500

#define DISPLAY_WIDTH   240
#define DISPLAY_HEIGHT  240
#define DISPLAY_MIRROR_X false
#define DISPLAY_MIRROR_Y false
#define DISPLAY_SWAP_XY false

#define DISPLAY_OFFSET_X  0
#define DISPLAY_OFFSET_Y  0

#define DISPLAY_DC_GPIO     GPIO_NUM_43
#define DISPLAY_CS_GPIO     GPIO_NUM_44
#define DISPLAY_CLK_GPIO    GPIO_NUM_21
#define DISPLAY_MOSI_GPIO   GPIO_NUM_47
#define DISPLAY_RST_GPIO    GPIO_NUM_NC

#define DISPLAY_BACKLIGHT_PIN GPIO_NUM_46
#define DISPLAY_BACKLIGHT_OUTPUT_INVERT false

#define UART_ECHO_TXD GPIO_NUM_38
#define UART_ECHO_RXD GPIO_NUM_48
#define UART_ECHO_RTS (-1)
#define UART_ECHO_CTS (-1)

#define MOTOR_SPEED_MAX 100
#define MOTOR_SPEED_80  80
#define MOTOR_SPEED_60  60
#define MOTOR_SPEED_MIN 0

#define ECHO_UART_PORT_NUM      UART_NUM_1
#define ECHO_UART_BAUD_RATE     (115200)
#define BUF_SIZE                (1024)

/*
   电池电压检测
   主体 PCB 上电池电压经 100k/100k 分压后送入 ADC2_CH3(GPIO14)，
   参数与官方出厂 demo（example/factory_demo_v1/main/app/app_power.c）一致。
*/
#define BATTERY_ADC_UNIT        ADC_UNIT_2
#define BATTERY_ADC_CHANNEL     ADC_CHANNEL_3   // ESP32-S3: ADC2_CH3 = GPIO14
#define BATTERY_UPPER_RESISTOR  100000.0f       // 分压上电阻 100k
#define BATTERY_LOWER_RESISTOR  100000.0f       // 分压下电阻 100k
// 充电检测引脚：主体没有引出充电状态脚，先留空，充电状态由电压趋势软件判断
#define BATTERY_CHARGING_PIN    GPIO_NUM_NC

typedef enum {
    LIGHT_MODE_CHARGING_BREATH = 0,
    LIGHT_MODE_POWER_LOW,
    LIGHT_MODE_ALWAYS_ON,
    LIGHT_MODE_BLINK,
    LIGHT_MODE_WHITE_BREATH_SLOW,
    LIGHT_MODE_WHITE_BREATH_FAST,
    LIGHT_MODE_FLOWING,
    LIGHT_MODE_SHOW,
    LIGHT_MODE_SLEEP,
    LIGHT_MODE_MAX
} light_mode_t;

/* Camera PINs*/
#define SPARKBOT_CAMERA_XCLK      (GPIO_NUM_15)
#define SPARKBOT_CAMERA_PCLK      (GPIO_NUM_13)
#define SPARKBOT_CAMERA_VSYNC     (GPIO_NUM_6)
#define SPARKBOT_CAMERA_HSYNC     (GPIO_NUM_7)
#define SPARKBOT_CAMERA_D0        (GPIO_NUM_11)
#define SPARKBOT_CAMERA_D1        (GPIO_NUM_9)
#define SPARKBOT_CAMERA_D2        (GPIO_NUM_8)
#define SPARKBOT_CAMERA_D3        (GPIO_NUM_10)
#define SPARKBOT_CAMERA_D4        (GPIO_NUM_12)
#define SPARKBOT_CAMERA_D5        (GPIO_NUM_18)
#define SPARKBOT_CAMERA_D6        (GPIO_NUM_17)
#define SPARKBOT_CAMERA_D7        (GPIO_NUM_16)

#define SPARKBOT_CAMERA_PWDN      (GPIO_NUM_NC)
#define SPARKBOT_CAMERA_RESET     (GPIO_NUM_NC)
#define SPARKBOT_CAMERA_XCLK      (GPIO_NUM_15)
#define SPARKBOT_CAMERA_PCLK      (GPIO_NUM_13)
#define SPARKBOT_CAMERA_VSYNC     (GPIO_NUM_6)
#define SPARKBOT_CAMERA_HSYNC     (GPIO_NUM_7)

#define SPARKBOT_CAMERA_XCLK_FREQ (16000000)
#define SPARKBOT_LEDC_TIMER       (LEDC_TIMER_0)
#define SPARKBOT_LEDC_CHANNEL     (LEDC_CHANNEL_0)

#define SPARKBOT_CAMERA_SIOD      (GPIO_NUM_NC)
#define SPARKBOT_CAMERA_SIOC      (GPIO_NUM_NC)

#endif // _BOARD_CONFIG_H_
