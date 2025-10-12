#include "webrtc_gst_cpp/gst_webrtc_adapter.h"
#include "webrtc_gst_cpp/signaling_client.h"

#include <gst/gst.h>
#include <gst/webrtc/webrtc.h>
#include <gst/app/gstappsrc.h>
#include <gst/sdp/gstsdpmessage.h>

#include <nlohmann/json.hpp> // ensure nlohmann-json3-dev installed
#include <thread>
#include <iostream>

using json = nlohmann::json;

struct GstWebRTCAdapter::Impl {
  std::string signaling_url;
  int width;
  int height;
  int fps;

  GMainLoop* loop = nullptr;
  GstElement* pipeline = nullptr;
  GstElement* appsrc = nullptr;
  GstElement* webrtc = nullptr;

  std::unique_ptr<SignalingClient> signaling;

  std::thread glib_thread;

  std::mutex mutex;

  Impl(const std::string& url, int w, int h, int f) : signaling_url(url), width(w), height(h), fps(f) {}
};

static void on_offer_created(GstPromise *promise, gpointer user_data);
static void on_answer_created(GstPromise *promise, gpointer user_data);

// GStreamer signal callbacks
static void on_negotiation_needed(GstElement *webrtc, gpointer user_data) {
  GstPromise *promise = gst_promise_new_with_change_func(on_offer_created, user_data, nullptr);
  g_signal_emit_by_name(webrtc, "create-offer", nullptr, promise);
}

static void on_ice_candidate(GstElement *webrtc, guint mlineindex, gchar *candidate, gpointer user_data) {
  GstWebRTCAdapter::Impl* self = static_cast<GstWebRTCAdapter::Impl*>(user_data);
  json j;
  j["type"] = "candidate";
  j["candidate"] = candidate;
  j["sdpMLineIndex"] = (int)mlineindex;
  j["sdpMid"] = "0";
  if (self->signaling) self->signaling->Send(j.dump());
}

static void on_offer_created(GstPromise *promise, gpointer user_data) {
  GstWebRTCAdapter::Impl* self = static_cast<GstWebRTCAdapter::Impl*>(user_data);
  GstWebRTCSessionDescription* offer = nullptr;
  const GstStructure *reply = gst_promise_get_reply(promise);
  GstWebRTCSessionDescription *offer_desc = nullptr;

  // Extract SDP from promise
  GstWebRTCSessionDescription *offer_sdp = nullptr;
  GstStructure *s = nullptr;
  gsize len = 0;
  const gchar *sdp_text = nullptr;

  // The reply contains a GstWebRTCSessionDescription under key "offer"
  GValueArray *array = nullptr;
  GValue val = G_VALUE_INIT;
  if (gst_structure_has_field(reply, "offer")) {
    const GValue *value = gst_structure_get_value(reply, "offer");
    GstWebRTCSessionDescription *sdesc = (GstWebRTCSessionDescription*)g_value_get_boxed(value);
    if (sdesc) {
      // Convert SDP to text
      GstSDPMessage *sdp = sdesc->sdp;
      gchar *sdp_str = gst_sdp_message_as_text(sdp);
      json j;
      j["type"] = "offer";
      j["sdp"] = std::string(sdp_str);
      g_free(sdp_str);
      if (self->signaling) self->signaling->Send(j.dump());
    }
  }

  gst_promise_unref(promise);
}

GstWebRTCAdapter::GstWebRTCAdapter(const std::string& signaling_url, int width, int height, int fps)
  : impl_(new Impl(signaling_url, width, height, fps)) {}

GstWebRTCAdapter::~GstWebRTCAdapter() {
  if (impl_) {
    if (impl_->pipeline) {
      gst_element_set_state(impl_->pipeline, GST_STATE_NULL);
      gst_object_unref(impl_->pipeline);
      impl_->pipeline = nullptr;
    }
    if (impl_->loop) {
      g_main_loop_quit(impl_->loop);
      if (impl_->glib_thread.joinable()) impl_->glib_thread.join();
      g_main_loop_unref(impl_->loop);
      impl_->loop = nullptr;
    }
  }
}

// helper: parse incoming SDP and set as remote description
static void set_remote_description_from_sdp(GstElement* webrtc, const std::string& sdp_text, GstWebRTCSDPType type) {
  GError *err = nullptr;
  GstSDPMessage *sdp = nullptr;
  if (gst_sdp_message_new_from_text(const_cast<gchar*>(sdp_text.c_str()), &sdp) != GST_SDP_OK) {
    std::cerr << "Failed to parse SDP text\n";
    return;
  }
  GstWebRTCSessionDescription *answer = gst_webrtc_session_description_new(type, sdp);
  GstPromise *promise = gst_promise_new();
  g_signal_emit_by_name(webrtc, "set-remote-description", answer, promise);
  gst_promise_interrupt(promise);
  gst_promise_unref(promise);
  gst_webrtc_session_description_free(answer);
}

bool GstWebRTCAdapter::Start() {
  std::lock_guard<std::mutex> lock(impl_->mutex);

  gst_init(nullptr, nullptr);

  impl_->loop = g_main_loop_new(nullptr, FALSE);

  // build pipeline: appsrc -> videoconvert -> queue -> vp8enc -> rtpvp8pay -> webrtcbin
  std::string pipeline_str = 
    "appsrc name=src is-live=true block=true format=time caps=video/x-raw,format=BGR,width=" +
    std::to_string(impl_->width) + ",height=" + std::to_string(impl_->height) + ",framerate=" + std::to_string(impl_->fps) + "/1 "
    "! videoconvert ! queue ! vp8enc deadline=1 cpu-used=5 error-resilient=1 keyframe-max-dist=60 "
    "! rtpvp8pay ! queue ! application/x-rtp,media=video,encoding-name=VP8,payload=96 ! webrtcbin name=webrtc";

  GError *error = nullptr;
  impl_->pipeline = gst_parse_launch(pipeline_str.c_str(), &error);
  if (!impl_->pipeline) {
    std::cerr << "Failed to create pipeline: " << (error ? error->message : "unknown") << std::endl;
    return false;
  }

  impl_->appsrc = gst_bin_get_by_name(GST_BIN(impl_->pipeline), "src");
  impl_->webrtc = gst_bin_get_by_name(GST_BIN(impl_->pipeline), "webrtc");

  if (!impl_->appsrc || !impl_->webrtc) {
    std::cerr << "Failed to get appsrc or webrtc element from pipeline\n";
    return false;
  }

  g_signal_connect(impl_->webrtc, "on-negotiation-needed", G_CALLBACK(on_negotiation_needed), impl_.get());
  g_signal_connect(impl_->webrtc, "on-ice-candidate", G_CALLBACK(on_ice_candidate), impl_.get());

  gst_element_set_state(impl_->pipeline, GST_STATE_PLAYING);

  // start GLib main loop in background thread (webrtcbin uses the main context)
  impl_->glib_thread = std::thread([this]() {
    g_main_loop_run(impl_->loop);
  });

  // Setup signaling
  impl_->signaling = std::make_unique<SignalingClient>(impl_->signaling_url);
  impl_->signaling->SetOnOpen([this]() {
    std::cout << "[signaling] connected\n";
  });

  impl_->signaling->SetOnMessage([this](const std::string& msg) {
    this->OnSignalingMessage(msg);
  });

  impl_->signaling->Connect();

  return true;
}

void GstWebRTCAdapter::PushFrame(const cv::Mat& bgr_frame) {
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->appsrc) return;

  // Ensure frame size matches expected
  if (bgr_frame.cols != impl_->width || bgr_frame.rows != impl_->height) {
    // do not resize here; upstream node should resize. But we'll handle quickly:
    cv::Mat resized;
    cv::resize(bgr_frame, resized, cv::Size(impl_->width, impl_->height));
    // proceed with resized
    GstBuffer *buffer = gst_buffer_new_allocate(NULL, resized.total() * resized.elemSize(), NULL);
    gst_buffer_fill(buffer, 0, resized.data, resized.total() * resized.elemSize());
    GST_BUFFER_PTS(buffer) = gst_util_uint64_scale(gst_clock_get_time(gst_system_clock_obtain()), GST_SECOND, GST_SECOND);
    GstFlowReturn ret = GST_FLOW_OK;
    g_signal_emit_by_name(impl_->appsrc, "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);
    if (ret != GST_FLOW_OK) {
      std::cerr << "appsrc push-buffer returned " << ret << std::endl;
    }
    return;
  }

  GstBuffer *buffer = gst_buffer_new_allocate(NULL, bgr_frame.total() * bgr_frame.elemSize(), NULL);
  gst_buffer_fill(buffer, 0, bgr_frame.data, bgr_frame.total() * bgr_frame.elemSize());
  GST_BUFFER_PTS(buffer) = gst_util_uint64_scale(gst_clock_get_time(gst_system_clock_obtain()), GST_SECOND, GST_SECOND);
  GstFlowReturn ret = GST_FLOW_OK;
  g_signal_emit_by_name(impl_->appsrc, "push-buffer", buffer, &ret);
  gst_buffer_unref(buffer);
  if (ret != GST_FLOW_OK) {
    std::cerr << "appsrc push-buffer returned " << ret << std::endl;
  }
}

void GstWebRTCAdapter::OnSignalingMessage(const std::string& msg) {
  // parse JSON message
  try {
    json j = json::parse(msg);
    std::string type = j.value("type", "");
    if (type == "answer") {
      std::string sdp = j.value("sdp", "");
      // set remote description (answer)
      set_remote_description_from_sdp(impl_->webrtc, sdp, GST_WEBRTC_SDP_TYPE_ANSWER);
      std::cout << "[signaling] answer applied\n";
    } else if (type == "offer") {
      std::string sdp = j.value("sdp", "");
      // set remote and create answer
      set_remote_description_from_sdp(impl_->webrtc, sdp, GST_WEBRTC_SDP_TYPE_OFFER);
      // create answer
      GstPromise *promise = gst_promise_new_with_change_func([](GstPromise *p, gpointer user_data){
        GstWebRTCAdapter::Impl* self = static_cast<GstWebRTCAdapter::Impl*>(user_data);
        const GstStructure *reply = gst_promise_get_reply(p);
        if (gst_structure_has_field(reply, "answer")) {
          const GValue *val = gst_structure_get_value(reply, "answer");
          GstWebRTCSessionDescription *answer = (GstWebRTCSessionDescription*)g_value_get_boxed(val);
          GstSDPMessage *sdp = answer->sdp;
          gchar *sdp_str = gst_sdp_message_as_text(sdp);
          json out;
          out["type"] = "answer";
          out["sdp"] = std::string(sdp_str);
          g_free(sdp_str);
          // set-local-description
          GstPromise *p2 = gst_promise_new();
          g_signal_emit_by_name(self->webrtc, "set-local-description", answer, p2);
          gst_promise_unref(p2);
          if (self->signaling) self->signaling->Send(out.dump());
        }
        gst_promise_unref(p);
      }, impl_.get());
      g_signal_emit_by_name(impl_->webrtc, "create-answer", nullptr, promise);
    } else if (type == "candidate") {
      std::string candidate = j.value("candidate", "");
      int mline = j.value("sdpMLineIndex", 0);
      g_signal_emit_by_name(impl_->webrtc, "add-ice-candidate", "0", mline, candidate.c_str());
    } else {
      // ignore
    }
  } catch (const std::exception& e) {
    std::cerr << "Failed to parse signaling message: " << e.what() << std::endl;
  }
}