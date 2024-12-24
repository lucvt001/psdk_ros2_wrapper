#include <psdk_wrapper/flight_controller.hpp>

FlightControllerWrapper* FlightControllerWrapper::instance_ = nullptr;
FlightControllerWrapper::FlightControllerWrapper(std::shared_ptr<rclcpp::Node> node)
    : node_(node)
{
    instance_ = this;
    T_DjiReturnCode returnCode = DjiTest_FlightControlInit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        log_error(node_, "Init flight control wrapper failed");
        return;
    }
    else
        log_info(node_, "Init flight control wrapper success");

    returnCode = DjiFlightController_RegJoystickCtrlAuthorityEventCallback(
        &FlightControllerWrapper::JoystickCtrlAuthSwitchEventCallbackStatic);

    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS && returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_NONSUPPORT) {
        log_error(node_, "Register joystick control authority event callback failed");
        return;
    }

    std::string node_name = node_->get_name();

    takeoff_action_server_ = rclcpp_action::create_server<TakeOff>
    (
        node_, node_name + "/takeoff_action",
        [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const TakeOff::Goal> goal) {
            RCLCPP_INFO(node_->get_logger(), "Received takeoff goal request");
            (void)uuid;
            // Accept the goal
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<TakeOff>> goal_handle) {
            RCLCPP_INFO(node_->get_logger(), "Received request to cancel takeoff goal");
            // Accept the cancel request
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<TakeOff>> goal_handle) {
            // Execute the goal in a separate thread
            std::thread([this, goal_handle]() {
                execute_takeoff(goal_handle);
            }).detach();
        }
    );

    land_action_server_ = rclcpp_action::create_server<Land>
    (
        node_, node_name + "/land_action",
        [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Land::Goal> goal) {
            RCLCPP_INFO(node_->get_logger(), "Received land goal request");
            (void)uuid;
            // Accept the goal
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Land>> goal_handle) {
            RCLCPP_INFO(node_->get_logger(), "Received request to cancel land goal");
            // Accept the cancel request
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<Land>> goal_handle) {
            // Execute the goal in a separate thread
            std::thread([this, goal_handle]() {
                execute_land(goal_handle);
            }).detach();
        }
    );

    move_to_position_action_server_ = rclcpp_action::create_server<MoveToPosition>
    (
        node_, node_name + "/move_to_position_action",
        [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const MoveToPosition::Goal> goal) {
            RCLCPP_INFO(node_->get_logger(), "Received move to position goal request");
            (void)uuid;
            // Accept the goal
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToPosition>> goal_handle) {
            RCLCPP_INFO(node_->get_logger(), "Received request to cancel move to position goal");
            // Accept the cancel request
            return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToPosition>> goal_handle) {
            // Execute the goal in a separate thread
            std::thread([this, goal_handle]() {
                execute_move_to_position(goal_handle);
            }).detach();
        }
    );

    obtain_joystick_authority_service_ = node_->create_service<ObtainJoystickAuthority>(
        node_name + "/obtain_joystick_authority_service",
        [this](const std::shared_ptr<ObtainJoystickAuthority::Request> request,
               std::shared_ptr<ObtainJoystickAuthority::Response> response) {
            handle_obtain_joystick_authority(request, response);
        }
    );

    release_joystick_authority_service_ = node_->create_service<ReleaseJoystickAuthority>(
        node_name + "/release_joystick_authority_service",
        [this](const std::shared_ptr<ReleaseJoystickAuthority::Request> request,
               std::shared_ptr<ReleaseJoystickAuthority::Response> response) {
            handle_release_joystick_authority(request, response);
        }
    );

    set_joystick_mode_service_ = node_->create_service<SetJoystickMode>(
        node_name + "/set_joystick_mode_service",
        [this](const std::shared_ptr<SetJoystickMode::Request> request,
               std::shared_ptr<SetJoystickMode::Response> response) {
            handle_set_joystick_mode(request, response);
        }
    );

    joystick_command_subscriber_ = node_->create_subscription<JoystickCommand>(
        node_name + "/joystick_command",
        10,
        std::bind(&FlightControllerWrapper::joystick_command_callback, this, std::placeholders::_1)
    );

    velocity_command_subscriber_ = node_->create_subscription<VelocityCommand>(
        node_name + "/velocity_command",
        10,
        std::bind(&FlightControllerWrapper::velocity_command_callback, this, std::placeholders::_1)
    );
}


void FlightControllerWrapper::execute_takeoff(const std::shared_ptr<rclcpp_action::ServerGoalHandle<TakeOff>> goal_handle)
{
    log_info(node_, "Taking off...");

    auto result = std::make_shared<TakeOff::Result>();

    if (!DjiTest_FlightControlMonitoredTakeoff()) {
        log_error(node_, "Takeoff failed");
        goal_handle->abort(result);
    } else {
        log_info(node_, "Takeoff succeeded");
        goal_handle->succeed(result);
    }
}


void FlightControllerWrapper::execute_land(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Land>> goal_handle)
{
    log_info(node_, "Landing...");

    auto result = std::make_shared<Land::Result>();

    if (!DjiTest_FlightControlMonitoredLanding()) {
        log_error(node_, "Land failed");
        goal_handle->abort(result);
    } else {
        log_info(node_, "Land succeeded");
        goal_handle->succeed(result);
    }
}

void FlightControllerWrapper::execute_move_to_position(const std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveToPosition>> goal_handle)
{
    auto goal = goal_handle->get_goal();
    float x = goal->x;
    float y = goal->y;
    float z = goal->z;
    float yaw = goal->yaw;
    std::stringstream ss;
    ss << "Moving to position: x=" << x << ", y=" << y << ", z=" << z << ", yaw=" << yaw;
    log_info(node_, ss.str().c_str());
    auto result = std::make_shared<MoveToPosition::Result>();

    // if (!DjiTest_FlightControlMoveByPositionOffset(
    //         (T_DjiTestFlightControlVector3f) {x, y, z}, 
    //         yaw, 0.8, 1)) 
    // {
    //     log_error(node_, "Move to position failed");
    //     goal_handle->abort(result);
    // } else {
    //     log_info(node_, "Move to position succeeded");
    //     goal_handle->succeed(result);
    // }

    auto timeout_duration = std::chrono::seconds(goal->timeout);
    auto future = std::async(std::launch::async, [this, x, y, z, yaw]() {
        return DjiTest_FlightControlMoveByPositionOffset(     // This function will implicitly set joystick mode to all position control
            (T_DjiTestFlightControlVector3f) {x, y, z}, 
            yaw, 0.8, 1);
    });

    if (future.wait_for(timeout_duration) == std::future_status::timeout) {
        if (current_control_authority_ == DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_RC) {
            log_error(node_, "Move to position failed: RC took over control authority.");
            result->error_code = 2;
            goal_handle->abort(result);
        } else if (current_control_authority_ == DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_MSDK) {
            log_error(node_, "Move to position failed: MSDK took over control authority.");
            result->error_code = 2;
            goal_handle->abort(result);
        } else if (current_control_authority_ == DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_INTERNAL) {
            log_error(node_, "Move to position failed: Internal took over control authority.");
            result->error_code = 2;
            goal_handle->abort(result);
        } else {
            log_error(node_, "Move to position failed due to timeout.");
            result->error_code = 1;
            goal_handle->abort(result);
        }
    } else if (!future.get()) {
        log_error(node_, "Move to position failed due to other errors.");
        result->error_code = 3;
        goal_handle->abort(result);
    } else {
        log_info(node_, "Move to position succeeded");
        result->error_code = 0;
        goal_handle->succeed(result);
    }
}

void FlightControllerWrapper::handle_obtain_joystick_authority(const std::shared_ptr<ObtainJoystickAuthority::Request> request,
                                                               std::shared_ptr<ObtainJoystickAuthority::Response> response)
{
    T_DjiReturnCode returnCode = DjiFlightController_ObtainJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        log_error(node_, "Obtain joystick authority failed");
        response->is_success = false;
    } else {
        log_info(node_, "Obtained joystick authority");
        response->is_success = true;
    }
}

void FlightControllerWrapper::handle_release_joystick_authority(const std::shared_ptr<ReleaseJoystickAuthority::Request> request,
                                                                std::shared_ptr<ReleaseJoystickAuthority::Response> response)
{
    T_DjiReturnCode returnCode = DjiFlightController_ReleaseJoystickCtrlAuthority();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        log_error(node_, "Obtain joystick authority failed");
        response->is_success = false;
    } else {
        log_info(node_, "Obtained joystick authority");
        response->is_success = true;
    }
}

void FlightControllerWrapper::handle_set_joystick_mode(const std::shared_ptr<SetJoystickMode::Request> request,
                                                       std::shared_ptr<SetJoystickMode::Response> response)
{
    T_DjiFlightControllerJoystickMode joystickMode = {
        static_cast<E_DjiFlightControllerHorizontalControlMode>(request->horizontal_control_mode),
        static_cast<E_DjiFlightControllerVerticalControlMode>(request->vertical_control_mode),
        static_cast<E_DjiFlightControllerYawControlMode>(request->yaw_control_mode),
        static_cast<E_DjiFlightControllerHorizontalCoordinate>(request->horizontal_coordinate),
        static_cast<E_DjiFlightControllerStableControlMode>(request->stable_control_mode)
    };
    DjiFlightController_SetJoystickMode(joystickMode);
    log_info(node_, "Set joystick mode success");
    response->is_success = true;
}

void FlightControllerWrapper::joystick_command_callback(const JoystickCommand::SharedPtr msg)
{
    T_DjiFlightControllerJoystickCommand joystickCommand = {
        msg->x,
        msg->y,
        msg->z,
        msg->yaw
    };
    DjiFlightController_ExecuteJoystickAction(joystickCommand);
}

void FlightControllerWrapper::velocity_command_callback(const VelocityCommand::SharedPtr msg)
{
    float x = msg->x;   // m/s
    float y = msg->y;   // m/s
    float z = msg->z;   // m/s
    float yaw = msg->yaw;   // deg/s
    DjiTest_FlightControlVelocityAndYawRateCtrl((T_DjiTestFlightControlVector3f) {x, y, z}, yaw, 100);
}

T_DjiReturnCode FlightControllerWrapper::JoystickCtrlAuthSwitchEventCallbackStatic(T_DjiFlightControllerJoystickCtrlAuthorityEventInfo eventData) {
    // Boilerplate function to call non-static member function
    return instance_->JoystickCtrlAuthSwitchEventCallback(eventData);
}

T_DjiReturnCode FlightControllerWrapper::JoystickCtrlAuthSwitchEventCallback(T_DjiFlightControllerJoystickCtrlAuthorityEventInfo eventData) 
{
    current_control_authority_ = eventData.curJoystickCtrlAuthority;
    switch (eventData.joystickCtrlAuthoritySwitchEvent) {
        case DJI_FLIGHT_CONTROLLER_MSDK_GET_JOYSTICK_CTRL_AUTH_EVENT: {
            if (current_control_authority_ == DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_MSDK) {
                log_info(node_, "[Event]Msdk request to obtain joystick ctrl authority\r\n");
            } else {
                log_info(node_, "[Event]Msdk request to release joystick ctrl authority\r\n");
            }
            break;
        }
        case DJI_FLIGHT_CONTROLLER_INTERNAL_GET_JOYSTICK_CTRL_AUTH_EVENT: {
            if (current_control_authority_ == DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_INTERNAL) {
                log_info(node_, "[Event]Internal request to obtain joystick ctrl authority\r\n");
            } else {
                log_info(node_, "[Event]Internal request to release joystick ctrl authority\r\n");
            }
            break;
        }
        case DJI_FLIGHT_CONTROLLER_OSDK_GET_JOYSTICK_CTRL_AUTH_EVENT: {
            if (current_control_authority_ == DJI_FLIGHT_CONTROLLER_JOYSTICK_CTRL_AUTHORITY_OSDK) {
                log_info(node_, "[Event] PSDK request to obtain joystick ctrl authority\r\n");
            } else {
                log_info(node_, "[Event] PSDK request to release joystick ctrl authority\r\n");
            }
            break;
        }
        case DJI_FLIGHT_CONTROLLER_RC_LOST_GET_JOYSTICK_CTRL_AUTH_EVENT :
            log_info(node_, "[Event]Current joystick ctrl authority is reset to rc due to rc lost\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_RC_NOT_P_MODE_RESET_JOYSTICK_CTRL_AUTH_EVENT :
            log_info(node_, "[Event]Current joystick ctrl authority is reset to rc for rc is not in P mode\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_RC_SWITCH_MODE_GET_JOYSTICK_CTRL_AUTH_EVENT :
            log_info(node_, "[Event]Current joystick ctrl authority is reset to rc due to rc switching mode\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_RC_PAUSE_GET_JOYSTICK_CTRL_AUTH_EVENT :
            log_info(node_, "[Event]Current joystick ctrl authority is reset to rc due to rc pausing\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_RC_REQUEST_GO_HOME_GET_JOYSTICK_CTRL_AUTH_EVENT :
            log_info(node_, "[Event]Current joystick ctrl authority is reset to rc due to rc request for return\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_LOW_BATTERY_GO_HOME_RESET_JOYSTICK_CTRL_AUTH_EVENT :
            log_info(node_, "[Event]Current joystick ctrl authority is reset to rc for low battery return\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_LOW_BATTERY_LANDING_RESET_JOYSTICK_CTRL_AUTH_EVENT :
            log_info(node_, "[Event]Current joystick ctrl authority is reset to rc for low battery land\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_OSDK_LOST_GET_JOYSTICK_CTRL_AUTH_EVENT:
            log_info(node_, "[Event]Current joystick ctrl authority is reset to rc due to sdk lost\r\n");
            break;
        case DJI_FLIGHT_CONTROLLER_NERA_FLIGHT_BOUNDARY_RESET_JOYSTICK_CTRL_AUTH_EVENT :
            log_info(node_, "[Event]Current joystick ctrl authority is reset to rc due to near boundary\r\n");
            break;
        default:
            log_info(node_, "[Event]Unknown joystick ctrl authority event\r\n");
    }

    return DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS;
}

FlightControllerWrapper::~FlightControllerWrapper()
{
    T_DjiReturnCode returnCode = DjiTest_FlightControlDeInit();
    if (returnCode != DJI_ERROR_SYSTEM_MODULE_CODE_SUCCESS) {
        log_error(node_, "Deinit flight control wrapper failed");
    } else
        log_info(node_, "Deinit flight control wrapper success");
}