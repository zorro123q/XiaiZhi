#ifndef CAMERA_H
#define CAMERA_H

#include <string>

class Camera {
public:
    virtual void SetExplainUrl(const std::string& url, const std::string& token) = 0;
    // 设置本地人脸识别服务地址（默认空实现，不支持该能力的实现可忽略）
    virtual void SetFaceRecognizeUrl(const std::string& url) {}
    virtual bool Capture() = 0;
    virtual bool SetHMirror(bool enabled) = 0;
    virtual bool SetVFlip(bool enabled) = 0;
    virtual std::string Explain(const std::string& question) = 0;
    // 识别人物身份（默认不支持）
    virtual std::string IdentifyPerson(const std::string& question) {
        return "{\"success\": false, \"message\": \"Face recognition not supported\"}";
    }
};

#endif // CAMERA_H
