#include <iostream>

#include "depthai/depthai.hpp"

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

// Structs

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

struct point2i {
  int x;
  int y;
};


// typedef point2i aruco_to_grid_idx[4];
// struct aruco_to_grid_idx{}

// Global objects - these shouldn't be here but they are so easy

struct StereoCameraCalibrationWrapper our_wrap;

xrt_vec2 uv_lut[21][19];

cv::Ptr<cv::aruco::Dictionary> our_dict;

// Functions




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

std::vector<std::vector<point2i>>

draw_arucos_2(bool offsX, bool offsY, cv::Mat our_mat,
                   cv::Ptr<cv::aruco::Dictionary> our_dict) {


  std::vector<std::vector<point2i>> out;

  int side = 80;


  int xoff_px = side; // Get it one grid cell away from very edge
  int yoff_px = side; // Get it one grid cell away from very edge

  int xoff_idx = 1;
  int yoff_idx = 1;

  if (offsX) {
    xoff_px += side;
    xoff_idx ++;
  }
  if (offsY) {
    yoff_px += side;
    yoff_idx ++;
  }

  int arucoIdx = 0;


  for (int y = 0; y < 10-1; y++) {
    for (int x = 0; x < 9-1; x++) {
      // float top_left_px = xoff + x * 2 * side;

      // float screen_u_top = / (1440.0)

      cv::Rect rect(xoff_px + (x * 2 * side), yoff_px + (y * 2 * side), side, side);
      cv::Mat tmp(side, side, CV_8UC1);
      printf("%d %d\n", x * side, y * side);
      cv::aruco::drawMarker(our_dict, arucoIdx++, side, tmp, 1);

      // Flip over vertical axis - we're undoing what the combiner/reflector/whatever-you-call-it did so that the arucos can be detected.
      // Note that the screen is also physically rotated 180 degrees, but we are *not* accounting for that - if we write our code correctly that won't matter.
      // All the flips are confusing - try to let ur intuition take care of it instead
      cv::flip(tmp, our_mat(rect), 0);
      
      std::vector<point2i> in;
      
      // Order is weird because screen flip - I got these empircally by looking, not through thinking about it

      // 0 = (0, down)
      // 1 = (right, down)
      // 2 = (right, 0)
      // 3 = (0, 0)

      in.push_back({xoff_idx + x*2, yoff_idx + y*2 + 1});
      in.push_back({xoff_idx + x*2 + 1, yoff_idx + y*2 + 1});
      in.push_back({xoff_idx + x*2 + 1, yoff_idx + y*2});
      in.push_back({xoff_idx + x*2, yoff_idx + y*2});
      out.push_back(in);
    }
  }
  return out;
}

void draw_arucos_and_capture(bool offsL, bool offsT) {

  cv::Mat our_mat(1600, 1440 * 2, CV_8UC1);
  our_mat = cv::Scalar(255);
  draw_arucos_2(0, 0, our_mat, our_dict);
  draw_arucos_2(1440, 0, our_mat, our_dict);

}

void debug_aruco_corner_order(cv::Mat img, std::vector<std::vector<cv::Point2f>> corners, std::vector<int> ids) {
  if (corners.size() == 0) {
    return;
  }
  cv::putText(img, "0", corners[0][0], cv::FONT_HERSHEY_SIMPLEX, 1, {255, 0, 0});
  cv::putText(img, "1", corners[0][1], cv::FONT_HERSHEY_SIMPLEX, 1, {255, 0, 0});
  cv::putText(img, "2", corners[0][2], cv::FONT_HERSHEY_SIMPLEX, 1, {255, 0, 0});
  cv::putText(img, "3", corners[0][3], cv::FONT_HERSHEY_SIMPLEX, 1, {255, 0, 0});
}

void debug_aruco_corners(cv::Mat img, std::vector<std::vector<cv::Point2f>> all_corners, const std::vector<int> ids, const std::vector<std::vector<point2i>> corner_to_grid_pt) {
  if (all_corners.size() == 0) {
    return;
  }
  for (int i = 0; i < ids.size(); i++) {
    int id = ids[i];
    // std::vector<cv::Point2f>& corners = all_corners[i];
    std::vector<point2i> map = corner_to_grid_pt[id];
    
    for (int j = 0; j < 4; j++) {
      int x_pt = map[j].x;
      int y_pt = map[j].y;
      char bob[64];
      sprintf(bob, "(%d, %d)", x_pt, y_pt);
      cv::putText(img, bob, all_corners[i][j], cv::FONT_HERSHEY_SIMPLEX, .5, {255, 0, 0});
    }
  }
  // cv::putText(img, "0", corners[0][0], cv::FONT_HERSHEY_SIMPLEX, 1, {255, 0, 0});
  // cv::putText(img, "1", corners[0][1], cv::FONT_HERSHEY_SIMPLEX, 1, {255, 0, 0});
  // cv::putText(img, "2", corners[0][2], cv::FONT_HERSHEY_SIMPLEX, 1, {255, 0, 0});
  // cv::putText(img, "3", corners[0][3], cv::FONT_HERSHEY_SIMPLEX, 1, {255, 0, 0});
}

int main() {
  fill_lut();
  dummy_fill_wrap();

  cv::namedWindow("hi", 0);
  cv::moveWindow("hi", 8000, 0);
  cv::setWindowProperty("hi", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

  cv::Ptr<cv::aruco::Dictionary> our_dict =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);

  cv::Mat our_mat(1600, 1440 * 2, CV_8UC1);
  our_mat = cv::Scalar(255);

  std::vector<std::vector<point2i>> grid_pts = draw_arucos_2(false, false, our_mat(cv::Rect(0,0,1440,1600)), our_dict);
  draw_arucos_2(false, false, our_mat(cv::Rect(1440,0,1440,1600)), our_dict);

  // cv::Mat corners;
  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<int> ids;
  // cv::Mat ids;

  cv::aruco::detectMarkers(our_mat, our_dict, corners, ids);

  cv::Mat out;
  cv::cvtColor(our_mat, out, cv::COLOR_GRAY2BGR);

  // cv::aruco::drawDetectedMarkers(out, corners, ids);

  cv::imshow("hi", out);
  cv::waitKey(1);

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

      // cv::aruco::drawDetectedMarkers(hey, corners, ids);
      debug_aruco_corners(hey, corners, ids, grid_pts);
      cv::imshow("right", hey);
    }

    int key = cv::waitKey(1);
    if (key == 'q' || key == 'Q') {
      return 0;
    }
  }
  return 0;
}