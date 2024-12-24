#include <psdk_wrapper/gimbal.hpp>

GimbalWrapper::GimbalWrapper(std::shared_ptr<rclcpp::Node> node) : node_(node)
{
    node_->declare_parameter("set_gimbal_angle_server_name", "/psdk_wrapper_node/set_gimbal_angle_service");
    node_->get_parameter("set_gimbal_angle_server_name", set_gimbal_angle_server_name_);

    set_gimbal_angle_srv_ = node_->create_service<SetGimbalAngle>(
        set_gimbal_angle_server_name_,
        std::bind(&GimbalWrapper::handle_set_gimbal_angle, this, std::placeholders::_1, std::placeholders::_2)
    );

    T_DjiReturnCode returnCode = DjiGimbalManager_Init();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        log_error(node_, "Init gimbal manager failed");
    } else {
        log_info(node_, "Init gimbal manager success");
    }

    returnCode = DjiGimbalManager_SetMode(DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, DJI_GIMBAL_MODE_FREE);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        log_error(node_, "Set gimbal mode failed");
    } else
        log_info(node_, "Set gimbal mode success");

    returnCode = DjiGimbalManager_Reset(DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, DJI_GIMBAL_RESET_MODE_PITCH_AND_YAW);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        log_error(node_, "Reset gimbal failed");
    }
}


void GimbalWrapper::handle_set_gimbal_angle(const std::shared_ptr<SetGimbalAngle::Request> request, 
                                            std::shared_ptr<SetGimbalAngle::Response> response)
{
    float pitch = request->pitch;
    float roll = request->roll;
    float yaw = request->yaw;
    T_DjiGimbalManagerRotation rotation = (T_DjiGimbalManagerRotation) 
        {DJI_GIMBAL_ROTATION_MODE_RELATIVE_ANGLE, pitch, roll, yaw, 0.5};   // 0.5s is estimated time
    T_DjiReturnCode returnCode = DjiGimbalManager_Rotate(DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, rotation);
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        log_error(node_, "Set angle failed");
        response->is_success = false;
    } else {
        log_info(node_, "Set angle success");
        response->is_success = true;
    }
}
GimbalWrapper::~GimbalWrapper()
{
    log_info(node_, "Deinit gimbal manager and reset gimbal angle");
    T_DjiReturnCode returnCode = DjiGimbalManager_Reset(DJI_MOUNT_POSITION_PAYLOAD_PORT_NO1, DJI_GIMBAL_RESET_MODE_PITCH_AND_YAW);
    returnCode = DjiGimbalManager_Deinit();
}