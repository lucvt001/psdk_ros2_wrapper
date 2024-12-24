#ifndef PSDK_WRAPPER_HPP
#define PSDK_WRAPPER_HPP

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <psdk_wrapper/application.hpp>
#include <psdk_wrapper/liveview.hpp>
#include <psdk_wrapper/flight_controller.hpp>
#include <psdk_wrapper/gimbal.hpp>

class PSDKWrapper : public rclcpp::Node
{
public:
    PSDKWrapper();
    ~PSDKWrapper();

    // initialize has to be a separate function to avoid "bad_weak_ptr" error when passing the node to LiveViewWrapper
    void initialize();

private:
    bool is_enable_liveview_;
    bool is_enable_flight_control_;
    bool is_enable_gimbal_;

    std::unique_ptr<LiveViewWrapper> liveview_wrapper_;
    std::unique_ptr<FlightControllerWrapper> flight_controller_wrapper_;
    std::unique_ptr<GimbalWrapper> gimbal_wrapper_;
};

#endif // PSDK_WRAPPER_HPP