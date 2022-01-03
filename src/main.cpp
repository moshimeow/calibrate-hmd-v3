#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/opencv.hpp>

void draw_arucos(int xoff, int yoff, cv::Mat our_mat,
                 cv::Ptr<cv::aruco::Dictionary> our_dict) {

  int arucoIdx = 0;

  for (int x = 1; x < 18; x++) {
    for (int y = 1; y < 20; y++) {
      printf("%d %d\n", x * 80, y * 80);
      cv::aruco::drawMarker(
          our_dict, arucoIdx++, 40,
          our_mat(cv::Rect(xoff + x * 80, yoff + y * 80, 40, 40)), 1);
    }
  }
  // cv::imshow("hi", our_mat);
  // cv::waitKey(0);
}

void draw_arucos_2(int xoff, int yoff, cv::Mat our_mat,
                   cv::Ptr<cv::aruco::Dictionary> our_dict) {

  int arucoIdx = 0;

  int side = 80;

  for (int y = 1; y < 10; y++) {
    for (int x = 1; x < 9; x++) {
      // float top_left_px = xoff + x * 2 * side;

      // float screen_u_top = / (1440.0)

      cv::Rect rect(xoff + x * 2 * side, yoff + y * 2 * side, side, side);
      // cv::Mat tmp(side, side, CV_8UC1);
      printf("%d %d\n", x * side, y * side);
      cv::aruco::drawMarker(our_dict, arucoIdx++, side, our_mat(rect), 1);
      // Flip over vertical axis - we're undoing what the mirror did so that the arucos can be detected.
      // Note that the screen is also physically rotated 180 degrees, but we are *not* accounting for that - if we write our code correctly that won't matter.
      // All the flips are confusing - try to let ur intuition take care of it instead
      // cv::flip(our_mat(rect), our_mat(rect), 1);
    }
  }
  // cv::imshow("hi", our_mat);
  // cv::waitKey(0);
}

int main() {
  // cv::aruco::Dictionary our_dict(cv::aruco::DICT_4X4_1000);

  // cv::
  // 1440x1600:
  // 40 px wide; 36x40 grid.
  // cv::Mat our_mat(1440, 1600, CV_8UC1);

  // cv::aruco::drawMarker(our_dict, 0, 100, our_mat(cv::Rect(0, 0, 100, 100)),
  // 1); cv::Mat dips_mat(256, 256, CV_8UC1); dips_mat = cv::Scalar(255);

  // our_mat.copyTo(dips_mat(cv::Rect(0, 0, our_mat.cols, our_mat.rows)));
  // cv::copyTo(our_mat, dips_mat,
  // );

  cv::namedWindow("hi", 0);
  cv::moveWindow("hi", 8000, 0);
  cv::setWindowProperty("hi", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

  cv::Ptr<cv::aruco::Dictionary> our_dict =
      cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);

  cv::Mat our_mat(1600, 1440 * 2, CV_8UC1);
  our_mat = cv::Scalar(255);
  draw_arucos_2(0, 0, our_mat, our_dict);
  draw_arucos_2(1440, 0, our_mat, our_dict);

  // cv::flip(our_mat, our_mat, 0);

  // cv::Mat corners;
  std::vector<std::vector<cv::Point2f>> corners;
  std::vector<int> ids;
  // cv::Mat ids;

  cv::aruco::detectMarkers(our_mat, our_dict, corners, ids);

  cv::Mat out;
  cv::cvtColor(our_mat, out, cv::COLOR_GRAY2BGR);

  cv::aruco::drawDetectedMarkers(out, corners, ids);

  cv::imshow("hi", out);
  cv::waitKey(0);

  // cv::aruco::drawDetectedMarkers

  // draw_arucos(-40, 0, our_mat);
  // draw_arucos(0, -40, our_mat);
  // draw_arucos(-40, -40, our_mat);
}