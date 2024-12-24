#include <psdk_wrapper/psdk_wrapper_node.hpp>
#include <cstring>

PSDKWrapper::PSDKWrapper() : Node("psdk_wrapper_node")
{
    this->declare_parameter<bool>("liveview.is_enable", false);
    this->declare_parameter<bool>("flight_control.is_enable", true);
    this->declare_parameter<bool>("gimbal.is_enable", true);

    this->get_parameter("liveview.is_enable", is_enable_liveview_);
    this->get_parameter("flight_control.is_enable", is_enable_flight_control_);
    this->get_parameter("gimbal.is_enable", is_enable_gimbal_);
}

void PSDKWrapper::initialize()
{

    if (is_enable_flight_control_) {
        flight_controller_wrapper_ = std::make_unique<FlightControllerWrapper>(this->shared_from_this());
        RCLCPP_INFO(get_logger(), "Flight control enabled");
    }

    if (is_enable_gimbal_) {
        gimbal_wrapper_ = std::make_unique<GimbalWrapper>(this->shared_from_this());
        RCLCPP_INFO(get_logger(), "Gimbal enabled");
    }
    
    if (is_enable_liveview_) {
        liveview_wrapper_ = std::make_unique<LiveViewWrapper>(this->shared_from_this());
        RCLCPP_INFO(get_logger(), "Liveview enabled");
    }
}

PSDKWrapper::~PSDKWrapper()
{
    std::cout << "Destructor called" << std::endl;
}

int main(int argc, char **argv)
{
    // Initialize DJICore before starting the ROS2 node
    Application application(argc, argv);

    // Other components of DJI will be initialized inside the node
    rclcpp::init(argc, argv);
    auto psdk_wrapper = std::make_shared<PSDKWrapper>();
    psdk_wrapper->initialize();

    rclcpp::spin(psdk_wrapper);
    rclcpp::shutdown();
    return 0;
}

