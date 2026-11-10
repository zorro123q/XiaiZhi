#ifndef ESP32_CAMERA_H
#define ESP32_CAMERA_H

#include <esp_camera.h>
#include <lvgl.h>
#include <thread>
#include <memory>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "camera.h"

struct JpegChunk {
    uint8_t* data;
    size_t len;
};

class Esp32Camera : public Camera {
private:
    camera_fb_t* fb_ = nullptr;
    lv_img_dsc_t preview_image_;
    std::string explain_url_;
    std::string explain_token_;
    std::string face_url_;      // 本地人脸识别服务地址
    std::thread encoder_thread_;

    // 把当前帧 POST 到指定 URL（Explain 与 IdentifyPerson 共用）
    std::string PostImage(const std::string& url, const std::string& question);

public:
    Esp32Camera(const camera_config_t& config);
    ~Esp32Camera();

    virtual void SetExplainUrl(const std::string& url, const std::string& token);
    virtual void SetFaceRecognizeUrl(const std::string& url) override;
    virtual bool Capture();
    // 翻转控制函数
    virtual bool SetHMirror(bool enabled) override;
    virtual bool SetVFlip(bool enabled) override;
    virtual std::string Explain(const std::string& question);
    virtual std::string IdentifyPerson(const std::string& question) override;
    // 抓一帧转 JPEG（网页实时预览用），返回 malloc 缓冲区，调用方 free
    virtual uint8_t* SnapshotJpeg(size_t* out_len, int quality = 70) override;
};

#endif // ESP32_CAMERA_H