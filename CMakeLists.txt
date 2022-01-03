# Copyright 2018-2021, Collabora, Ltd. SPDX-License-Identifier: BSL-1.0

cmake_minimum_required(VERSION 3.10.2)
project(aruco-calibrate VERSION 21.0.0)

# CMake 3.11 introduced CMP0072 - Prefer GLVND
if(POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)
endif()

# We use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#
# Dependencies
#
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/sanitizers")
include(CMakeDependentOption)
include(GNUInstallDirs)
if(NOT ${CMAKE_VERSION} VERSION_LESS 3.9)
  include(CheckIPOSupported)
  check_ipo_supported(RESULT HAS_IPO)
endif()

# Android SDK doesn't look for 3.8 and 3.9, which is what new distros ship with.
set(Python_ADDITIONAL_VERSIONS 3.8 3.9)
if(NOT CMAKE_VERSION VERSION_LESS 3.12)
  find_package(Python3 REQUIRED Interpreter)
  set(PYTHON_EXECUTABLE Python3::Interpreter)
else()
  find_program(PYTHON_EXECUTABLE python3)
  if(PYTHON_EXECUTABLE MATCHES "WindowsApps")
    # If you hit this error, you will have to install Python 3 or try harder to
    # tell CMake where it is.
    message(
      FATAL_ERROR
        "Found WindowsApps alias for Python. Make sure Python3 is installed, then choose 'Manage App Execution Aliases' in Start and disable the aliases for Python."
    )
  endif()
endif()

# Redundant mention of version is required because module defaults to looking
# for 2.91-compatible, which the config file for a 3.x says it's not compatible
# with. find_package(Eigen3 3 REQUIRED)
find_package(
  OpenCV
  COMPONENTS core
             aruco
             calib3d
             highgui
             imgproc
             imgcodecs
             features2d
             video
             CONFIG)
# find_package(JPEG)
find_package(depthai CONFIG)
# find_package(SDL2 CONFIG) find_package(ZLIB) find_package(cJSON)

find_package(Sanitizers)

add_library(xrt-pthreads INTERFACE)
if(WIN32)
  find_package(pthreads_windows REQUIRED)
  target_link_libraries(xrt-pthreads INTERFACE PThreads4W::PThreads4W_CXXEXC)
else()
  set(CMAKE_THREAD_PREFER_PTHREAD ON)
  find_package(Threads)
  target_link_libraries(xrt-pthreads INTERFACE Threads::Threads)
endif()

if(NOT ANDROID)
  # @TODO Turn into a find_package LIBUVC file.
  # pkg_check_modules(LIBUVC libuvc)

  # @TODO Turn into a find_package FFMPEG file.
  # pkg_check_modules(FFMPEG libavcodec)
endif()

# This one is named differently because that's what CTest uses
option(BUILD_TESTING "Enable building of the test suite?" ON)

option(XRT_FEATURE_COLOR_LOG "Enable logging in color on supported platforms"
       ON)
option(XRT_FEATURE_TRACING "Enable debug tracing on supported platforms" OFF)

cmake_dependent_option(
  CMAKE_INTERPROCEDURAL_OPTIMIZATION
  "Enable inter-procedural (link-time) optimization" OFF "HAS_IPO" OFF)
cmake_dependent_option(
  XRT_HAVE_WAYLAND "Enable Wayland support" ON
  "WAYLAND_FOUND AND WAYLAND_SCANNER_FOUND AND WAYLAND_PROTOCOLS_FOUND" OFF)
cmake_dependent_option(XRT_HAVE_XLIB "Enable xlib support" ON "X11_FOUND" OFF)
cmake_dependent_option(XRT_HAVE_XRANDR "Enable xlib-xrandr support" ON
                       "XRANDR_FOUND" OFF)
cmake_dependent_option(XRT_HAVE_XCB "Enable xcb support" ON "XCB_FOUND" OFF)

cmake_dependent_option(
  XRT_HAVE_VULKAN
  "Enable Vulkan Graphics API support (also needed for compositor)" ON
  "VULKAN_FOUND" OFF)
cmake_dependent_option(XRT_HAVE_OPENGL "Enable OpenGL Graphics API support" ON
                       "OPENGL_FOUND" OFF)
cmake_dependent_option(
  XRT_HAVE_OPENGLES "Enable OpenGL-ES Graphics API support" ON "OpenGLES_FOUND"
  OFF)
cmake_dependent_option(XRT_HAVE_EGL "Enable OpenGL on EGL Graphics API support"
                       ON "EGL_FOUND; XRT_HAVE_OPENGL OR XRT_HAVE_OPENGLES" OFF)
cmake_dependent_option(XRT_HAVE_DBUS "Enable dbus support (for BLE support)" ON
                       "DBUS_FOUND" OFF)
cmake_dependent_option(
  XRT_FEATURE_COMPOSITOR_MAIN "Build main compositor host functionality" ON
  "XRT_HAVE_VULKAN; XRT_HAVE_WAYLAND OR XRT_HAVE_XCB OR ANDROID OR WIN32" OFF)
cmake_dependent_option(XRT_HAVE_LIBBSD "Enable libbsd support" ON
                       "LIBBSD_FOUND" OFF)
cmake_dependent_option(XRT_FEATURE_OPENXR "Build OpenXR runtime target" ON
                       "XRT_FEATURE_COMPOSITOR_MAIN" OFF)
cmake_dependent_option(
  XRT_FEATURE_SERVICE "Enable separate service module for OpenXR runtime" ON
  "NOT WIN32 AND XRT_FEATURE_OPENXR" OFF)
cmake_dependent_option(
  XRT_HAVE_SYSTEMD "Enable systemd support (for socket activation of service)"
  ON "Systemd_FOUND AND XRT_FEATURE_SERVICE" OFF)
cmake_dependent_option(
  XRT_INSTALL_SYSTEMD_UNIT_FILES
  "Install user unit files for systemd socket activation on installation" ON
  "XRT_HAVE_SYSTEMD" OFF)
cmake_dependent_option(
  XRT_INSTALL_ABSOLUTE_SYSTEMD_UNIT_FILES
  "Use an absolute path to monado-system in installed user unit files for systemd socket activation"
  ON
  "XRT_INSTALL_SYSTEMD_UNIT_FILES"
  OFF)
cmake_dependent_option(XRT_FEATURE_STEAMVR_PLUGIN "Build SteamVR plugin" ON
                       "NOT ANDROID" OFF)

if(NOT DEFINED XRT_FEATURE_OPENXR_LAYER_DEPTH)
  set(XRT_FEATURE_OPENXR_LAYER_DEPTH ON)
endif()
if(NOT DEFINED XRT_FEATURE_OPENXR_LAYER_CUBE)
  set(XRT_FEATURE_OPENXR_LAYER_CUBE OFF)
endif()
if(NOT DEFINED XRT_FEATURE_OPENXR_LAYER_CYLINDER)
  set(XRT_FEATURE_OPENXR_LAYER_CYLINDER ON)
endif()
if(NOT DEFINED XRT_FEATURE_OPENXR_LAYER_EQUIRECT2)
  set(XRT_FEATURE_OPENXR_LAYER_EQUIRECT2 ON)
endif()
if(NOT DEFINED XRT_FEATURE_OPENXR_LAYER_EQUIRECT1)
  set(XRT_FEATURE_OPENXR_LAYER_EQUIRECT1 ON)
endif()

# Most users won't touch these.
mark_as_advanced(XRT_FEATURE_COMPOSITOR_MAIN XRT_FEATURE_OPENXR)

# ILLIXR
set(ILLIXR_PATH
    ""
    CACHE PATH "Path to ILLIXR headers")

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  set(XRT_HAVE_LIBUDEV ON)
  set(XRT_HAVE_INTERNAL_HID ON)
else()
  cmake_dependent_option(
    XRT_HAVE_LIBUDEV "Enable libudev (used for device probing on Linux)" ON
    "UDEV_FOUND" OFF)
endif()
cmake_dependent_option(XRT_HAVE_LIBUSB "Enable libusb (used for most drivers)"
                       ON "LIBUSB1_FOUND" OFF)
cmake_dependent_option(
  XRT_HAVE_JPEG "Enable jpeg code (used for some video drivers)" ON
  "JPEG_FOUND" OFF)
cmake_dependent_option(XRT_HAVE_OPENCV "Enable OpenCV backend" ON
                       "OpenCV_FOUND" OFF)
cmake_dependent_option(XRT_HAVE_LIBUVC "Enable libuvc video driver" ON
                       "LIBUVC_FOUND AND XRT_HAVE_LIBUSB" OFF)
cmake_dependent_option(XRT_HAVE_FFMPEG "Enable ffmpeg testing video driver" ON
                       "FFMPEG_FOUND" OFF)
cmake_dependent_option(XRT_HAVE_SDL2 "Enable use of SDL2" ON
                       "SDL2_FOUND AND XRT_HAVE_OPENGL" OFF)
cmake_dependent_option(
  XRT_HAVE_SYSTEM_CJSON "Enable cJSON from system, instead of bundled source"
  ON "CJSON_FOUND" OFF)
cmake_dependent_option(XRT_HAVE_GST "Enable gstreamer" ON "GST_FOUND" OFF)
cmake_dependent_option(XRT_HAVE_PERCETTO "Enable percetto support" ON
                       "PERCETTO_FOUND" OFF)
cmake_dependent_option(XRT_HAVE_ONNXRUNTIME "Enable ONNX runtime support" ON
                       "ONNXRUNTIME_FOUND" OFF)

cmake_dependent_option(XRT_BUILD_DRIVER_PSVR "Enable PSVR HMD driver" ON
                       "HIDAPI_FOUND" OFF)
cmake_dependent_option(XRT_BUILD_DRIVER_RS "Enable RealSense device driver" ON
                       "realsense2_FOUND" OFF)
cmake_dependent_option(
  XRT_BUILD_DRIVER_VIVE
  "Enable driver for HTC Vive, Vive Pro, Valve Index, and their controllers" ON
  "ZLIB_FOUND AND XRT_HAVE_LINUX" OFF)
cmake_dependent_option(XRT_BUILD_DRIVER_OHMD "Enable OpenHMD driver" ON
                       "OPENHMD_FOUND" OFF)
cmake_dependent_option(
  XRT_BUILD_DRIVER_HANDTRACKING "Enable Camera Hand Tracking driver" ON
  "XRT_HAVE_ONNXRUNTIME" OFF)
cmake_dependent_option(
  XRT_BUILD_DRIVER_DAYDREAM
  "Enable the Google Daydream View controller driver (BLE, via D-Bus)" ON
  "XRT_HAVE_DBUS" OFF)
cmake_dependent_option(
  XRT_BUILD_DRIVER_ARDUINO "Enable Arduino input device with BLE via via D-Bus"
  ON "XRT_HAVE_DBUS" OFF)
cmake_dependent_option(XRT_BUILD_DRIVER_ILLIXR "Enable ILLIXR driver" ON
                       "ILLIXR_PATH" OFF)
option(XRT_BUILD_DRIVER_DUMMY "Enable dummy driver" ON)
cmake_dependent_option(XRT_BUILD_DRIVER_ULV2 "Enable Ultraleap v2 driver" ON
                       "LeapV2_FOUND" OFF)
cmake_dependent_option(XRT_BUILD_DRIVER_REMOTE "Enable remote debugging driver"
                       ON "XRT_HAVE_LINUX OR ANDROID" OFF)
option(XRT_BUILD_DRIVER_WMR "Enable Windows Mixed Reality driver" ON)

# These all use the Monado internal hid wrapper.
cmake_dependent_option(XRT_BUILD_DRIVER_HDK "Enable HDK driver" ON
                       "XRT_HAVE_INTERNAL_HID" OFF)
cmake_dependent_option(XRT_BUILD_DRIVER_PSMV "Enable Playstation Move driver"
                       ON "XRT_HAVE_INTERNAL_HID" OFF)
cmake_dependent_option(XRT_BUILD_DRIVER_HYDRA "Enable Hydra driver" ON
                       "XRT_HAVE_INTERNAL_HID" OFF)
cmake_dependent_option(XRT_BUILD_DRIVER_NS "Enable North Star driver" ON
                       "XRT_HAVE_INTERNAL_HID" OFF)
cmake_dependent_option(
  XRT_BUILD_DRIVER_VF
  "Build video frame driver (for video file support, uses gstreamer)" ON
  "XRT_HAVE_GST" OFF)
cmake_dependent_option(XRT_BUILD_DRIVER_DEPTHAI "DepthAI" ON "depthai_FOUND"
                       OFF)

# This one defaults to off, even if we find the deps.
cmake_dependent_option(XRT_BUILD_DRIVER_SURVIVE "Enable libsurvive driver" ON
                       "SURVIVE_FOUND" OFF)

cmake_dependent_option(XRT_BUILD_DRIVER_ANDROID "Enable Android sensors driver"
                       ON "ANDROID" OFF)
cmake_dependent_option(XRT_BUILD_DRIVER_QWERTY "Enable Qwerty driver" ON
                       "XRT_HAVE_SDL2" OFF)

# Default to PIC code
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Describe IPO setting
if(CMAKE_INTERPROCEDURAL_OPTIMIZATION)
  message(STATUS "Inter-procedural optimization enabled")
endif()

# Make sure we have pretty colours
if("${CMAKE_GENERATOR}" STREQUAL "Ninja")
  if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    add_compile_options(-fdiagnostics-color=always)
  elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    add_compile_options(-fcolor-diagnostics)
  endif()
endif()

#
# Decend into madness.
#

# add_library(cjson INTERFACE)
# target_include_directories(cjson INTERFACE src/cJSON)
# target_compile_definitions(cjson INTERFACE CJSON_HIDE_SYMBOLS)

add_executable(gui src/main.cpp)
target_link_libraries(gui PRIVATE ${OpenCV_LIBRARIES})
target_include_directories(gui PRIVATE ${OpenCV_INCLUDE_DIRS})

add_executable(view src/view.cpp)
target_link_libraries(view PRIVATE ${OpenCV_LIBRARIES} depthai::opencv depthai::core XLink )
target_include_directories(view PRIVATE ${OpenCV_INCLUDE_DIRS})


add_executable(capture src/capture.cpp)
target_link_libraries(capture PRIVATE ${OpenCV_LIBRARIES} depthai::opencv depthai::core XLink )
target_include_directories(capture PRIVATE ${OpenCV_INCLUDE_DIRS})
add_sanitizers(capture)


add_executable(calibrate src/defines.hpp src/fdg.cpp src/fdg.hpp src/calibrate.cpp src/cJSON.c src/cJSON.h)
target_link_libraries(calibrate PRIVATE ${OpenCV_LIBRARIES} depthai::opencv depthai::core XLink)
target_include_directories(calibrate PRIVATE ${OpenCV_INCLUDE_DIRS} src/cJSON)
add_sanitizers(calibrate)

add_executable(laplace src/laplace.cpp src/fdg.cpp src/fdg.hpp src/u_json.h src/u_json.c) # src/cJSON.c src/cJSON.h
target_link_libraries(laplace PRIVATE ${OpenCV_LIBRARIES} depthai::opencv depthai::core XLink)
target_include_directories(laplace PRIVATE ${OpenCV_INCLUDE_DIRS} src/cJSON)
add_sanitizers(laplace)
