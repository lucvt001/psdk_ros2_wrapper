#ifndef LIVEVIEW_HPP
#define LIVEVIEW_HPP

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <liveview/test_liveview.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <module_sample_c/utils/util_misc.h>

#include <psdk_interfaces/srv/enable_recording.hpp>
#include <psdk_wrapper/utils.h>

using EnableRecording = psdk_interfaces::srv::EnableRecording;

class LiveViewWrapper
{
public:
    explicit LiveViewWrapper(std::shared_ptr<rclcpp::Node> node);
    ~LiveViewWrapper();

private:
    std::shared_ptr<rclcpp::Node> node_;
    image_transport::Publisher image_pub_;
    rclcpp::Service<EnableRecording>::SharedPtr enable_recording_srv_;

    int camera_index_;
    std::string mp4_output_folder_;
    std::string topic_name_;
    bool is_display_;
    std::string enable_recording_server_name_;
    cv::VideoWriter video_writer_;
    int frame_width_ = -1;
    int frame_height_ = -1;
    bool is_recording_started_ = false;

    static void DjiUser_ShowRgbImageCallback(CameraRGBImage img, void *userData);
    void publishImage(cv::Mat &mat);
    void startCameraStream();
    void stopCameraStream();
    
    void handleEnableRecording(const std::shared_ptr<EnableRecording::Request> request, 
                                std::shared_ptr<EnableRecording::Response> response);

    LiveviewSample *liveviewSample;
};

#endif // LIVEVIEW_HPP