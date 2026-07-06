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
 * @brief Camera video stream publisher (H.264/RTP over UDP -> ROS2 image topics)
 */

#ifndef __FIXPOSITION_DRIVER_ROS2_CAMERA_HPP__
#define __FIXPOSITION_DRIVER_ROS2_CAMERA_HPP__

/* LIBC/STL */
#include <atomic>
#include <memory>
#include <string>
#include <thread>

/* EXTERNAL */
#include <fpsdk_ros2/ext/rclcpp.hpp>
#include <opencv2/videoio.hpp>

/* PACKAGE */
#include "params.hpp"
#include "ros2_msgs.hpp"

namespace fixposition {
/* ****************************************************************************************************************** */

/**
 * @brief Receives the sensor's camera video stream (H.264/RTP over UDP, via GStreamer/OpenCV) in a background
 *        thread and publishes it as sensor_msgs/Image (raw) and sensor_msgs/CompressedImage (JPEG).
 *
 * The sensor must be configured to stream the camera to this host (Camera Configuration -> Stream). The host
 * needs GStreamer H.264 support (packages gstreamer1.0-libav and gstreamer1.0-plugins-bad).
 */
class CameraPublisher {
   public:
    /**
     * @brief Constructor
     *
     * @param[in]  nh         Node handle
     * @param[in]  params     Driver parameters (uses the camera_* and reconnect_delay_ fields)
     * @param[in]  output_ns  Resolved output namespace for the topics
     * @param[in]  qos        QoS settings for the image publishers
     */
    CameraPublisher(std::shared_ptr<rclcpp::Node> nh, const DriverParams& params, const std::string& output_ns,
                    const rclcpp::QoS& qos);

    /**
     * @brief Destructor. Stops the worker thread if still running.
     */
    ~CameraPublisher();

    /**
     * @brief Start capturing and publishing (spawns the worker thread)
     *
     * @returns true on success
     */
    bool Start();

    /**
     * @brief Stop capturing and publishing (joins the worker thread)
     */
    void Stop();

   private:
    /**
     * @brief Worker thread: (re)opens the stream and publishes frames until stopped
     */
    void Run();

    /**
     * @brief Build the GStreamer pipeline string (from params_.camera_pipeline_ or the port)
     */
    std::string BuildPipeline() const;

    std::shared_ptr<rclcpp::Node> nh_;  //!< Node handle
    rclcpp::Logger logger_;             //!< Logger
    DriverParams params_;               //!< Driver parameters
    std::string frame_id_;              //!< frame_id for published images

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;                 //!< Raw image topic
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;  //!< Compressed (JPEG) image topic

    cv::VideoCapture cap_;      //!< OpenCV/GStreamer capture
    std::thread thread_;        //!< Worker thread
    std::atomic<bool> stop_;    //!< Stop flag for the worker thread
};

/* ****************************************************************************************************************** */
}  // namespace fixposition
#endif  // __FIXPOSITION_DRIVER_ROS2_CAMERA_HPP__
