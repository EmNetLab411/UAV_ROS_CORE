#ifndef GST_WEBRTC_ADAPTER_H
#define GST_WEBRTC_ADAPTER_H

#include <string>
#include <memory>
#include <mutex>
#include <opencv2/opencv.hpp>

class SignalingClient;

class GstWebRTCAdapter {
public:
  GstWebRTCAdapter(const std::string& signaling_url, int width = 640, int height = 480, int fps = 15);
  ~GstWebRTCAdapter();

  bool Start(); // initializes GStreamer pipeline and signaling
  void PushFrame(const cv::Mat& bgr_frame); // push frame into appsrc
  void OnSignalingMessage(const std::string& msg); // called by signaling client when a message arrives

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

#endif // GST_WEBRTC_ADAPTER_H