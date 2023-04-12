add_library(force_directed_graph STATIC fdg.cpp fdg.hpp)
target_link_libraries(force_directed_graph PRIVATE ${OpenCV_LIBRARIES} aux_util
aux_math)



add_executable(aruco_test aruco_test.cpp)
target_link_libraries(aruco_test PRIVATE ${OpenCV_LIBRARIES})
target_include_directories(aruco_test PRIVATE ${OpenCV_INCLUDE_DIRS})

# add_executable(view view.cpp) target_link_libraries( view PRIVATE
# ${OpenCV_LIBRARIES} depthai::core XLink aux_util aux_math xrt-external-cjson
# load_basalt_calibration) target_include_directories(view PRIVATE
# ${OpenCV_INCLUDE_DIRS})

add_executable(capture capture.cpp)
target_link_libraries(
  capture
  PRIVATE ${OpenCV_LIBRARIES}
          depthai::core
          XLink
          aux_util
          aux_math
          xrt-external-cjson
          load_basalt_calibration)
target_include_directories(capture PRIVATE ${OpenCV_INCLUDE_DIRS})
add_sanitizers(capture)

add_executable(calibrate defines.hpp #fdg.cpp fdg.hpp
                         calibrate.cpp)
target_link_libraries(
  calibrate
  PRIVATE ${OpenCV_LIBRARIES}
          depthai::core
          XLink
          aux_util
          aux_math
          xrt-external-cjson
          force_directed_graph
          load_basalt_calibration)
target_include_directories(calibrate PRIVATE ${OpenCV_INCLUDE_DIRS})
add_sanitizers(calibrate)

add_executable(laplace laplace.cpp ) # fdg.cpp fdg.hpp
target_link_libraries(
  laplace
  PRIVATE ${OpenCV_LIBRARIES}
          depthai::core
          XLink
          aux_util
          aux_math
          xrt-external-cjson
          force_directed_graph
          load_basalt_calibration)

target_include_directories(laplace PRIVATE ${OpenCV_INCLUDE_DIRS})
add_sanitizers(laplace)
