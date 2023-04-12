add_library(load_basalt_calibration STATIC load_basalt_calibration.cpp
                                           load_basalt_calibration.hpp)
target_link_libraries(load_basalt_calibration aux_util aux_os aux_tracking)
target_include_directories(load_basalt_calibration INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})