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
    // 拉取人脸库名单：GET <face_url 同目录>/list，返回给大模型直接念的中文
    virtual std::string ListKnownPeople() override;
    // 抓一帧转 JPEG（网页实时预览用），返回 malloc 缓冲区，调用方 free
    virtual uint8_t* SnapshotJpeg(size_t* out_len, int quality = 70) override;

    // 给"持续取帧"的功能（视觉跟随等）用：拿到预览缓冲的裸指针直接写像素，
    // 写完调 CommitPreview() 刷一次屏。比每帧走一遍 Capture() 便宜得多——
    // 那个会连取两帧、还会动 fb_ 成员。缓冲没分配时返回 nullptr。
    uint16_t* PreviewBuffer();
    void CommitPreview();
    // 收起预览，恢复表情显示
    void HidePreview();
};

#endif // ESP32_CAMERA_H