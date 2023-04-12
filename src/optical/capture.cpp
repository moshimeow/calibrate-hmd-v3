#include "util/u_time.h"
#include "os/os_time.h"

#include "depthai/depthai.hpp"

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <filesystem>

// Structs

struct point2i
{
	int x;
	int y;
};



struct capture_object
{
	cv::Ptr<cv::aruco::Dictionary> our_dict;

	cv::Mat gray_img_left = cv::Mat(cv::Size{1280, 800}, CV_8UC1);
	cv::Mat gray_img_right = cv::Mat(cv::Size{1280, 800}, CV_8UC1);

	dai::Device *device;

	std::shared_ptr<dai::DataOutputQueue> qLeft;
	std::shared_ptr<dai::DataOutputQueue> qRight;
};


// Functions

void
hack_get_images(capture_object *obj)
{
	// Very bad - I'm just too lazy to not do global state
	dai::ImgFrame *inLeft = obj->qLeft->get<dai::ImgFrame>().get();
	dai::ImgFrame *inRight = obj->qRight->get<dai::ImgFrame>().get();

	// Bad but will work. For later, use Monado frameservers or check the way depthai_driver does this referencing
	// the imgFrame
	memcpy(obj->gray_img_left.data, inLeft->getData().data(), 1280 * 800);
	memcpy(obj->gray_img_right.data, inRight->getData().data(), 1280 * 800);
}



std::vector<std::vector<point2i>>
draw_arucos_2(bool offsX, bool offsY, cv::Mat our_mat, cv::Ptr<cv::aruco::Dictionary> our_dict)
{

	std::vector<std::vector<point2i>> out;

	int side = 80;

	int xoff_px = side; // Get it one grid cell away from very edge
	int yoff_px = side; // Get it one grid cell away from very edge

	int xoff_idx = 1;
	int yoff_idx = 1;

	if (offsX) {
		xoff_px += side;
		xoff_idx++;
	}
	if (offsY) {
		yoff_px += side;
		yoff_idx++;
	}

	int arucoIdx = 0;

	for (int y = 0; y < 10 - 1; y++) {
		for (int x = 0; x < 9 - 1; x++) {
			// float top_left_px = xoff + x * 2 * side;

			// float screen_u_top = / (1440.0)

			cv::Rect rect(xoff_px + (x * 2 * side), yoff_px + (y * 2 * side), side, side);
			cv::Mat tmp(side, side, CV_8UC1);
			// printf("%d %d\n", x * side, y * side);
			cv::aruco::drawMarker(our_dict, arucoIdx++, side, tmp, 1);

			// Flip over vertical axis - we're undoing what the
			// combiner/reflector/whatever-you-call-it did so that the arucos can be
			// detected. Note that the screen is also physically rotated 180 degrees,
			// but we are *not* accounting for that - if we write our code correctly
			// that won't matter. All the flips are confusing - try to let ur
			// intuition take care of it instead
			cv::flip(tmp, our_mat(rect), 0);

			std::vector<point2i> in;

			// Order is weird because screen flip - I got these empircally by looking,
			// not through thinking about it

			// 0 = (0, down)
			// 1 = (right, down)
			// 2 = (right, 0)
			// 3 = (0, 0)

			in.push_back({xoff_idx + x * 2, yoff_idx + y * 2 + 1});
			in.push_back({xoff_idx + x * 2 + 1, yoff_idx + y * 2 + 1});
			in.push_back({xoff_idx + x * 2 + 1, yoff_idx + y * 2});
			in.push_back({xoff_idx + x * 2, yoff_idx + y * 2});
			out.push_back(in);
		}
	}
	return out;
}

void
debug_aruco_corner_order(cv::Mat img, std::vector<std::vector<cv::Point2f>> corners, std::vector<int> ids)
{
	if (corners.size() == 0) {
		return;
	}
	cv::putText(img, "0", corners[0][0], cv::FONT_HERSHEY_SIMPLEX, 1, {255, 0, 0});
	cv::putText(img, "1", corners[0][1], cv::FONT_HERSHEY_SIMPLEX, 1, {255, 0, 0});
	cv::putText(img, "2", corners[0][2], cv::FONT_HERSHEY_SIMPLEX, 1, {255, 0, 0});
	cv::putText(img, "3", corners[0][3], cv::FONT_HERSHEY_SIMPLEX, 1, {255, 0, 0});
}

void
debug_aruco_corners(cv::Mat img,
                    std::vector<std::vector<cv::Point2f>> all_corners,
                    const std::vector<int> ids,
                    const std::vector<std::vector<point2i>> corner_to_grid_pt)
{
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
}

void
setup_camera(capture_object *obj)
{
	// Create pipeline
	obj->device = new dai::Device();
	dai::Pipeline pipeline = {};

	// Define sources and outputs
	auto monoLeft = pipeline.create<dai::node::MonoCamera>();
	auto monoRight = pipeline.create<dai::node::MonoCamera>();
	auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
	auto xoutRight = pipeline.create<dai::node::XLinkOut>();

	xoutLeft->setStreamName("left");
	xoutRight->setStreamName("right");

	// Properties
	monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
	monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_800_P);
	monoLeft->setFps(60);
	monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);
	monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_800_P);
	monoLeft->setFps(60);

	// Linking
	monoRight->out.link(xoutRight->input);
	monoLeft->out.link(xoutLeft->input);

	// Connect to device and start pipeline
	obj->device->startPipeline(pipeline);

	// Output queues will be used to get the grayscale frames from the outputs
	// defined above
	obj->qLeft = obj->device->getOutputQueue("left", 2, false);
	obj->qRight = obj->device->getOutputQueue("right", 2, false);
}



int
main()
{
	std::filesystem::create_directories("captures");
	capture_object *the_one_and_only = new capture_object;
	setup_camera(the_one_and_only);
	cv::namedWindow("hi", 0);
	cv::moveWindow("hi", 8000, 0);
	cv::setWindowProperty("hi", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

	cv::Ptr<cv::aruco::Dictionary> our_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);

	std::vector<std::vector<int>> offsets_list = {
	    {0, 0}, //
	    {1, 0}, //
	    {0, 1}, //
	    {1, 1}  //
	};


	printf("waiting for initial exposure settle-down\r");
	fflush(stdout);

	os_nanosleep(U_TIME_1S_IN_NS * 5);

	bool first = true;

	for (std::vector<int> offsets : offsets_list) {

		cv::Mat our_mat(1600, 1440 * 2, CV_8UC1);
		our_mat = cv::Scalar(255);

		draw_arucos_2(offsets[0], offsets[1], our_mat(cv::Rect(0, 0, 1440, 1600)), our_dict);
		draw_arucos_2(offsets[0], offsets[1], our_mat(cv::Rect(1440, 0, 1440, 1600)), our_dict);

		cv::imshow("hi", our_mat);
		cv::waitKey(1);
		int i = 0;
		printf("\nmeow");
		fflush(stdout);
		for (int i = 0; i < 100; i++) {
			hack_get_images(the_one_and_only);

			cv::imshow("left", the_one_and_only->gray_img_left);
			cv::imshow("right", the_one_and_only->gray_img_right);
			int key = cv::waitKey(1);
			printf("\rWaiting (for exposure to settle down)");
			fflush(stdout);

			if (key == 'q' || key == 'Q') {
				return 0;
			}
		}

		for (int i = 0; i < 16; i++) {
			hack_get_images(the_one_and_only);



			cv::imshow("left", the_one_and_only->gray_img_left);
			cv::imshow("right", the_one_and_only->gray_img_right);
			int key = cv::waitKey(1);
			if (key == 'q' || key == 'Q') {
				return 0;
			}


			printf("\rCapturing");
			fflush(stdout);

			char name[1288];
			sprintf(name, "captures/%d%d_%d_left.png", offsets[0], offsets[1], i);
			cv::imwrite(name, the_one_and_only->gray_img_left);

			sprintf(name, "captures/%d%d_%d_right.png", offsets[0], offsets[1], i);
			cv::imwrite(name, the_one_and_only->gray_img_right);
		}
	}
	return 0;
}
