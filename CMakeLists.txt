# Copyright 2023, Pluto VR, Inc. SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.10.2)
project(linux-streaming VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Does not need to be fast and we're debugging a lot
set(CMAKE_BUILD_TYPE Debug)

# Make sure we have pretty colours
option(DISABLE_COLORED_OUTPUT
       "Always produce ANSI-colored output (GNU/Clang only)." OFF)

if(NOT DISABLE_COLORED_OUTPUT)
  if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    add_compile_options(-fdiagnostics-color=always)
  elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    add_compile_options(-fcolor-diagnostics)
  endif()
endif()

# Get Monado's CMake modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/monado/cmake")
list(APPEND CMAKE_MODULE_PATH
     "${CMAKE_CURRENT_SOURCE_DIR}/monado/cmake/sanitizers")

if(NOT MSVC)
  set(CMAKE_C_FLAGS
      "${CMAKE_C_FLAGS} -pedantic -Wall -Wextra -Wno-unused-parameter -Werror=incompatible-pointer-types"
  )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unused-parameter")
  set(CMAKE_SHARED_LINKER_FLAGS
      "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--no-undefined")
  set(CMAKE_MODULE_LINKER_FLAGS
      "${CMAKE_MODULE_LINKER_FLAGS} -Wl,--no-undefined")
endif()

# Default to PIC code
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(MONADO_SRC_XRT ${CMAKE_CURRENT_SOURCE_DIR}/monado/src/xrt)

# Set configuration options for Monado

# Helper to obey CMP0077 and work around warnings.
function(setForce NAME VALUE)
  set(${NAME}
      ${VALUE}
      CACHE INTERNAL "" FORCE)
endfunction()

# Let's use this for now: Monado's main compositor is really good and is an easy
# way to see that things are working.
setforce(XRT_FEATURE_COMPOSITOR_MAIN ON)

# No reason to disable this
setforce(XRT_FEATURE_COMPOSITOR_NULL ON)

# It looks like we can use Monado's normal out-of-proc OpenXR instance as-is.
# We'll see.
setforce(XRT_FEATURE_OPENXR ON)

setforce(XRT_FEATURE_RENDERDOC OFF) # we are not using this
setforce(XRT_FEATURE_STEAMVR_PLUGIN OFF) # we are not using this
setforce(XRT_FEATURE_TRACING OFF) # we are not using this yet
setforce(XRT_FEATURE_SLAM OFF) # we are not using this

setforce(XRT_FEATURE_CLIENT_DEBUG_GUI OFF) # we are not using this

setforce(XRT_MODULE_MONADO_CLI OFF) # we are not using this
setforce(XRT_MODULE_MONADO_GUI OFF) # we are not using this
setforce(XRT_MODULE_MERCURY_HANDTRACKING OFF)

setforce(XRT_HAVE_D3D11 OFF)
setforce(XRT_HAVE_D3D12 OFF)
setforce(XRT_HAVE_OPENCV ON)

# No drivers!
setforce(XRT_BUILD_DRIVER_ANDROID OFF)
setforce(XRT_BUILD_DRIVER_ARDUINO OFF)
setforce(XRT_BUILD_DRIVER_DAYDREAM OFF)
setforce(XRT_BUILD_DRIVER_DEPTHAI OFF)
setforce(XRT_BUILD_DRIVER_EUROC OFF)
setforce(XRT_BUILD_DRIVER_HANDTRACKING OFF)
setforce(XRT_BUILD_DRIVER_TWRAP OFF)
setforce(XRT_BUILD_DRIVER_HDK OFF)
setforce(XRT_BUILD_DRIVER_HYDRA OFF)
setforce(XRT_BUILD_DRIVER_ILLIXR OFF)
setforce(XRT_BUILD_DRIVER_NS OFF)
setforce(XRT_BUILD_DRIVER_OHMD OFF)
setforce(XRT_BUILD_DRIVER_OPENGLOVES OFF)
setforce(XRT_BUILD_DRIVER_PSMV OFF)
setforce(XRT_BUILD_DRIVER_PSVR OFF)
setforce(XRT_BUILD_DRIVER_QWERTY OFF)
setforce(XRT_BUILD_DRIVER_REALSENSE OFF)
setforce(XRT_BUILD_DRIVER_REMOTE OFF)
setforce(XRT_BUILD_DRIVER_RIFT_S OFF)
setforce(XRT_BUILD_DRIVER_SURVIVE OFF)
setforce(XRT_BUILD_DRIVER_ULV2 OFF)
setforce(XRT_BUILD_DRIVER_VF OFF)
setforce(XRT_BUILD_DRIVER_VIVE OFF)
setforce(XRT_BUILD_DRIVER_WMR OFF)
setforce(XRT_BUILD_DRIVER_SIMULAVR OFF)

# There is no clean way to disable the prober/builders (as far as I can see?) So
# we enable one driver so that the legacy builder does not die
setforce(XRT_BUILD_DRIVER_SIMULATED ON)

setforce(XRT_BUILD_SAMPLES OFF)

setforce(BUILD_TESTING OFF)
setforce(BUILD_DOC OFF)

add_subdirectory(monado)


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
find_package(depthai CONFIG)

find_package(Sanitizers)

add_subdirectory(src)


