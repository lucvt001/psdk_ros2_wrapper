#ifndef PSDK_WRAPPER__GIMBAL_WRAPPER_HPP_
#define PSDK_WRAPPER__GIMBAL_WRAPPER_HPP_

#include "dji_gimbal_manager.h"

#include <rclcpp/rclcpp.hpp>
#include <psdk_interfaces/srv/set_gimbal_angle.hpp>
#include <psdk_wrapper/log_all.hpp>

using SetGimbalAngle = psdk_interfaces::srv::SetGimbalAngle;

class GimbalWrapper
{
public:
    explicit GimbalWrapper(std::shared_ptr<rclcpp::Node> node);
    ~GimbalWrapper();

private:
    void handle_set_gimbal_angle(const std::shared_ptr<SetGimbalAngle::Request> request, 
                                std::shared_ptr<SetGimbalAngle::Response> response);

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Service<SetGimbalAngle>::SharedPtr set_gimbal_angle_srv_;

    std::string set_gimbal_angle_server_name_;
};

#endif  // PSDK_WRAPPER__GIMBAL_WRAPPER_HPP_