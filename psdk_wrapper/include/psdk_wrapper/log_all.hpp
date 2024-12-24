#ifndef LOG_ALL_HPP
#define LOG_ALL_HPP

#include <rclcpp/rclcpp.hpp>
#include <module_sample_c/widget/test_widget.h>
#include "dji_logger.h"

inline void log_info(const std::shared_ptr<rclcpp::Node>& node, const std::string& text)
{
    USER_LOG_INFO("%s", text.c_str());
    DjiTest_WidgetLogAppend(text.c_str());
    // RCLCPP_INFO(node->get_logger(), "%s", text.c_str());
};

inline void log_error(const std::shared_ptr<rclcpp::Node>& node, const std::string& text)
{
    USER_LOG_ERROR("%s", text.c_str());
    DjiTest_WidgetLogAppend(text.c_str());
    // RCLCPP_ERROR(node->get_logger(), "%s", text.c_str());
};

#endif // LOG_ALL_HPP