/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Camera video stream publisher (implementation)
 */

/* LIBC/STL */
#include <chrono>
#include <string>
#include <vector>

/* EXTERNAL */
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgcodecs.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>

/* PACKAGE */
#include "fixposition_driver_ros2/camera.hpp"

namespace fixposition {
/* ****************************************************************************************************************** */

CameraPublisher::CameraPublisher(std::shared_ptr<rclcpp::Node> nh, const DriverParams& params,
                                 const std::string& output_ns, const rclcpp::QoS& qos) /* clang-format off */ :
    nh_        { nh },
    logger_    { nh->get_logger() },
    params_    { params },
    frame_id_  { params.camera_frame_id_ },
    stop_      { false }  // clang-format on
{
    const std::string image_topic = output_ns + "/camera/image_raw";
    const std::string compressed_topic = image_topic + "/compressed";
    RCLCPP_INFO(logger_, "Advertise %s (sensor_msgs/Image)", image_topic.c_str());
    image_pub_ = nh_->create_publisher<sensor_msgs::msg::Image>(image_topic, qos);
    RCLCPP_INFO(logger_, "Advertise %s (sensor_msgs/CompressedImage)", compressed_topic.c_str());
    compressed_pub_ = nh_->create_publisher<sensor_msgs::msg::CompressedImage>(compressed_topic, qos);
}

CameraPublisher::~CameraPublisher() { Stop(); }

// ---------------------------------------------------------------------------------------------------------------------

bool CameraPublisher::Start() {
    stop_ = false;
    thread_ = std::thread(&CameraPublisher::Run, this);
    return true;
}

void CameraPublisher::Stop() {
    stop_ = true;
    if (thread_.joinable()) {
        thread_.join();
    }
    if (cap_.isOpened()) {
        cap_.release();
    }
    image_pub_.reset();
    compressed_pub_.reset();
}

// ---------------------------------------------------------------------------------------------------------------------

std::string CameraPublisher::BuildPipeline() const {
    // Explicit override wins
    if (!params_.camera_pipeline_.empty()) {
        return params_.camera_pipeline_;
    }
    // Default: unicast RTP/H.264 on the configured UDP port, decoded to raw frames for OpenCV's appsink.
    // Requires GStreamer plugins: -plugins-good (rtph264depay), -plugins-bad (h264parse), -libav (avdec_h264).
    return "udpsrc port=" + std::to_string(params_.camera_port_) +
           " caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, "
           "encoding-name=(string)H264, payload=(int)96\" "
           "! rtph264depay ! h264parse ! avdec_h264 ! videoconvert "
           "! appsink drop=true max-buffers=1 sync=false";
}

// ---------------------------------------------------------------------------------------------------------------------

void CameraPublisher::Run() {
    const std::string pipeline = BuildPipeline();
    RCLCPP_INFO(logger_, "Camera: GStreamer pipeline: %s", pipeline.c_str());

    const auto reconnect_delay = std::chrono::duration<double>(params_.reconnect_delay_);

    while (!stop_) {
        // (Re)open the stream if needed
        if (!cap_.isOpened()) {
            cap_.open(pipeline, cv::CAP_GSTREAMER);
            if (!cap_.isOpened()) {
                RCLCPP_WARN_THROTTLE(logger_, *nh_->get_clock(), 5000,
                                     "Camera: could not open GStreamer pipeline, retrying in %.1fs. Check that the "
                                     "H.264 decoder is installed (sudo apt install gstreamer1.0-libav "
                                     "gstreamer1.0-plugins-bad) and that the sensor streams to this host.",
                                     params_.reconnect_delay_);
                std::this_thread::sleep_for(reconnect_delay);
                continue;
            }
            RCLCPP_INFO(logger_, "Camera: stream opened");
        }

        // Grab a frame
        cv::Mat frame;
        if (!cap_.read(frame) || frame.empty()) {
            RCLCPP_WARN_THROTTLE(logger_, *nh_->get_clock(), 5000,
                                 "Camera: failed to read frame, reconnecting in %.1fs", params_.reconnect_delay_);
            cap_.release();
            std::this_thread::sleep_for(reconnect_delay);
            continue;
        }

        // Shared header
        std_msgs::msg::Header header;
        header.stamp = nh_->now();
        header.frame_id = frame_id_;

        const std::string encoding =
            (frame.channels() == 1) ? sensor_msgs::image_encodings::MONO8 : sensor_msgs::image_encodings::BGR8;

        // Raw image (only convert/publish if someone is listening)
        if (image_pub_->get_subscription_count() > 0) {
            image_pub_->publish(*cv_bridge::CvImage(header, encoding, frame).toImageMsg());
        }

        // Compressed image as JPEG (the expensive encode is skipped when there are no subscribers)
        if (compressed_pub_->get_subscription_count() > 0) {
            std::vector<unsigned char> buf;
            if (cv::imencode(".jpg", frame, buf)) {
                sensor_msgs::msg::CompressedImage compressed;
                compressed.header = header;
                compressed.format = "jpeg";
                compressed.data.assign(buf.begin(), buf.end());
                compressed_pub_->publish(compressed);
            } else {
                RCLCPP_WARN_THROTTLE(logger_, *nh_->get_clock(), 5000, "Camera: JPEG encoding failed");
            }
        }
    }

    if (cap_.isOpened()) {
        cap_.release();
    }
    RCLCPP_INFO(logger_, "Camera: stopped");
}

/* ****************************************************************************************************************** */
}  // namespace fixposition
