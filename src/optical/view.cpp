#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/opencv.hpp>

struct xrt_size {
  int w;
  int h;
};

struct Whatever {
  cv::Size image_size_pixels_cv;
  cv::Matx<double, 3, 3> cameraMatrix;
  cv::Matx<double, 4, 1> distortion_fisheye_mat;
  bool use_fisheye;
};

struct StereoCameraCalibrationWrapper {
  Whatever view[2];
  cv::Mat_<double> camera_translation_mat;
  cv::Mat_<double> camera_rotation_mat;
  cv::Mat_<double> camera_essential_mat;
  cv::Mat_<double> camera_fundamental_mat;
};

struct xrt_vec2 {
  float x;
  float y;
};

struct StereoCameraCalibrationWrapper our_wrap;

xrt_vec2 uv_lut[21][19];

void fill_lut() {
  for (int y = 0; y < 21; y++) {
    for (int x = 0; x < 19; x++) {
      uv_lut[y][x].x = (float)x / 18.0f;
      uv_lut[y][x].y = (float)y / 20.0f;
      printf("%f %f\n", uv_lut[y][x].x, uv_lut[y][x].y);
    }
  }
};

void dummy_fill_wrap() {

  our_wrap.view[0].image_size_pixels_cv.height = 800;
  our_wrap.view[0].image_size_pixels_cv.width = 1280;

  our_wrap.view[1].image_size_pixels_cv.height = 800;
  our_wrap.view[1].image_size_pixels_cv.width = 1280;

  our_wrap.view[0].cameraMatrix = {508.2431317803847, 0, 625.1262964163459,
                                   //
                                   0, 508.0092415823303, 436.0861794525318,
                                   //
                                   0, 0, 1}; // Shut up

  our_wrap.view[1].cameraMatrix = {508.0612269474984, 0, 646.8775239309249,
                                   //
                                   0, 507.794410062113, 348.0291105458621,
                                   //
                                   0, 0, 1}; // Shhhhhh

  our_wrap.view[0].distortion_fisheye_mat = {
      -0.01178353719932522, -0.002743341972778042, 5.13223456164071e-05,
      -6.336463736072942e-05};

  our_wrap.view[1].distortion_fisheye_mat = {
      -0.01308899660105348, 0.004549237696403933, -0.00751955708545033,
      0.00240647121573423};

  our_wrap.view[0].use_fisheye = true;
  our_wrap.view[1].use_fisheye = true;
}

int main() {
  fill_lut();
  dummy_fill_wrap();

  cv::Ptr<cv::aruco::Dictionary> our_dict =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);

  // Create pipeline
  dai::Pipeline pipeline;

  // Define sources and outputs
  auto monoLeft = pipeline.create<dai::node::MonoCamera>();
  auto monoRight = pipeline.create<dai::node::MonoCamera>();
  auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
  auto xoutRight = pipeline.create<dai::node::XLinkOut>();

  xoutLeft->setStreamName("left");
  xoutRight->setStreamName("right");

  // Properties
  monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
  monoLeft->setResolution(
      dai::MonoCameraProperties::SensorResolution::THE_800_P);
  monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
  monoRight->setResolution(
      dai::MonoCameraProperties::SensorResolution::THE_800_P);

  // Linking
  monoRight->out.link(xoutRight->input);
  monoLeft->out.link(xoutLeft->input);

  // Connect to device and start pipeline
  dai::Device device(pipeline);

  // Output queues will be used to get the grayscale frames from the outputs
  // defined above
  auto qLeft = device.getOutputQueue("left", 4, false);
  auto qRight = device.getOutputQueue("right", 4, false);

  while (true) {
    // Instead of get (blocking), we use tryGet (nonblocking) which will return
    // the available data or None otherwise
    auto inLeft = qLeft->tryGet<dai::ImgFrame>();
    auto inRight = qRight->tryGet<dai::ImgFrame>();

    if (inLeft) {
      cv::Mat hey = inLeft->getCvFrame();
      std::vector<std::vector<cv::Point2f>> corners;
      std::vector<int> ids;
      cv::aruco::detectMarkers(hey, our_dict, corners, ids);
      cv::cvtColor(hey, hey, cv::COLOR_GRAY2BGR);

      cv::aruco::drawDetectedMarkers(hey, corners, ids);
      cv::imshow("left", hey);
    }

    if (inRight) {
      cv::Mat hey = inRight->getCvFrame();
      std::vector<std::vector<cv::Point2f>> corners;
      std::vector<int> ids;
      cv::aruco::detectMarkers(hey, our_dict, corners, ids);
      cv::cvtColor(hey, hey, cv::COLOR_GRAY2BGR);

      cv::aruco::drawDetectedMarkers(hey, corners, ids);
      cv::imshow("right", hey);
    }

    int key = cv::waitKey(1);
    if (key == 'q' || key == 'Q') {
      return 0;
    }
  }
  return 0;
}