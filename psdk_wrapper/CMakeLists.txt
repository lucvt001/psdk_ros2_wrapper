cmake_minimum_required(VERSION 3.8)
project(psdk_wrapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -w)
endif()

set(USE_SYSTEM_ARCH LINUX)
add_definitions(-DSYSTEM_ARCH_LINUX)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

find_package(psdk_interfaces REQUIRED)

find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)


# Add path to FindFFMPEG.cmake and FindLIBUSB.cmake files
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/include/common/3rdparty)

find_package(FFMPEG REQUIRED)
find_package(LIBUSB REQUIRED)

set(DJI_OSAL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/include/common/osal)
set(DJI_HAL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/include/hal)
set(DJI_STREAMING_PATH ${CMAKE_CURRENT_SOURCE_DIR}/include/module_sample_cpp/liveview)
set(DJI_HMS_RETURN_CODES_PATH ${CMAKE_CURRENT_SOURCE_DIR}/include/module_sample_c/hms/data)
set(DJI_MODULE_SAMPLE_C_PATH ${CMAKE_CURRENT_SOURCE_DIR}/include/module_sample_c)

# Find toolchain name needed to select the right PSDK lib
execute_process(COMMAND uname -m
  OUTPUT_VARIABLE DEVICE_SYSTEM_ID)

if(DEVICE_SYSTEM_ID MATCHES x86_64)
  set(TOOLCHAIN_NAME x86_64-linux-gnu-gcc)
  add_definitions(-DPLATFORM_ARCH_x86_64=1)
elseif(DEVICE_SYSTEM_ID MATCHES aarch64)
  set(TOOLCHAIN_NAME aarch64-linux-gnu-gcc)
  add_definitions(-DPLATFORM_ARCH_aarch64=1)
else()
  message(FATAL_ERROR "FATAL: Please confirm your platform.")
endif()

# Check if PSDK lib is found
set(PSDK_LIB_PATH ${CMAKE_CURRENT_SOURCE_DIR}/psdk_lib/lib/${TOOLCHAIN_NAME})
find_library(PSDK_LIB NAMES libpayloadsdk.a PATHS ${PSDK_LIB_PATH})
if (PSDK_LIB)
  message(STATUS "PSDK library found: ${PSDK_LIB}")
else()
  message(FATAL_ERROR "PSDK library NOT found")
endif()

if (FFMPEG_FOUND)
    message(STATUS "Found FFMPEG installed in the system")
    message(STATUS " - Includes: ${FFMPEG_INCLUDE_DIR}")
    message(STATUS " - Libraries: ${FFMPEG_LIBRARIES}")

    EXECUTE_PROCESS(COMMAND ffmpeg -version
            OUTPUT_VARIABLE ffmpeg_version_psdk_libput
            OUTPUT_STRIP_TRAILING_WHITESPACE
            )
    string(REGEX MATCH "version.*Copyright" ffmpeg_version_line "${ffmpeg_version_psdk_libput}")
    string(REGEX MATCH " .* " ffmpeg_version "${ffmpeg_version_line}")
    string(REGEX MATCH "^ 5.*$" ffmpeg_major_version "${ffmpeg_version}")

    if (HEAD${ffmpeg_major_version} STREQUAL "HEAD")
        message(STATUS " - Version: ${ffmpeg_version}")
    else ()
        message(FATAL_ERROR " - Not support FFMPEG version: ${ffmpeg_major_version}, please install 4.x.x instead.")
    endif ()
    add_definitions(-DFFMPEG_INSTALLED)
else ()
    message(STATUS "Cannot Find FFMPEG")
endif (FFMPEG_FOUND)

if (LIBUSB_FOUND)
    message(STATUS "Found LIBUSB installed in the system")
    message(STATUS " - Includes: ${LIBUSB_INCLUDE_DIR}")
    message(STATUS " - Libraries: ${LIBUSB_LIBRARY}")

    add_definitions(-DLIBUSB_INSTALLED)
else ()
    message(STATUS "Cannot Find LIBUSB")
endif (LIBUSB_FOUND)

if (NOT EXECUTABLE_OUTPUT_PATH)
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
endif ()

include_directories(
  ${FFMPEG_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  include
  include/psdk_wrapper
  include/common
  include/hal
  include/module_sample_cpp
  include/module_sample_c
  ${CMAKE_CURRENT_SOURCE_DIR}/psdk_lib/include
)

set(ros_dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  sensor_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  cv_bridge
  image_transport
  psdk_interfaces
)

add_executable(psdk_wrapper_node
  src/application.cpp 
  src/psdk_wrapper_node.cpp 
  src/liveview.cpp
  src/utils.cpp
  src/flight_controller.cpp
  src/gimbal.cpp
  ${DJI_STREAMING_PATH}/test_liveview.cpp
  ${DJI_MODULE_SAMPLE_C_PATH}/flight_control/test_flight_control.c
)

file(GLOB 
  MODULE_SAMPLE_C_SOURCES
  ${DJI_MODULE_SAMPLE_C_PATH}/utils/*.c
  ${DJI_MODULE_SAMPLE_C_PATH}/widget/*.c
)

add_library(psdk_wrapper_libs SHARED
  ${DJI_OSAL_PATH}/osal_fs.c
  ${DJI_OSAL_PATH}/osal_socket.c
  ${DJI_OSAL_PATH}/osal.c
  ${DJI_HAL_PATH}/hal_network.c
  ${DJI_HAL_PATH}/hal_usb_bulk.c
  ${DJI_HAL_PATH}/hal_uart.c
  ${DJI_STREAMING_PATH}/dji_camera_image_handler.cpp
  ${DJI_STREAMING_PATH}/dji_camera_stream_decoder.cpp
  ${MODULE_SAMPLE_C_SOURCES}
)

ament_target_dependencies(psdk_wrapper_node
  ${ros_dependencies}
)

target_link_libraries(
  psdk_wrapper_libs
  ${PSDK_LIB}
)

target_link_libraries(
  psdk_wrapper_node
  psdk_wrapper_libs
  ${FFMPEG_LIBRARIES}
  ${OpenCV_LIBRARIES}
  -lusb-1.0
  -lavcodec
  -lswscale
)

install(TARGETS
  psdk_wrapper_node
  psdk_wrapper_libs
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  include/
  ${CMAKE_CURRENT_SOURCE_DIR}/psdk_lib/include/
  include/psdk_wrapper
  include/common
  include/hal
  include/module_sample_cpp
  include/module_sample_c
  DESTINATION include/
  FILES_MATCHING 
    PATTERN "*.h"
    PATTERN "*.hpp"
)

install(DIRECTORY
  config
  launch
  recordings
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  ${DJI_HMS_RETURN_CODES_PATH}/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_dependencies(
  "rclcpp"
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
