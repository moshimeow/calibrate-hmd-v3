#include <iostream>

#include "depthai/depthai.hpp"

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "tracking/t_calibration_opencv.hpp"

#include "cjson/cJSON.h"
#include "defines.hpp"
#include "fdg.hpp"

#include "load_basalt_calibration.hpp"

// Structs

// struct Whatever
// {
// 	cv::Size image_size_pixels_cv;
// 	cv::Matx<double, 3, 3> cameraMatrix;
// 	cv::Matx<double, 4, 1> distortion_fisheye_mat;
// 	bool use_fisheye;
// };

// struct StereoCameraCalibrationWrapper
// {
// 	Whatever view[2];
// 	cv::Mat_<double> camera_translation_mat;
// 	cv::Matx<double, 3, 3> camera_rotation_mat;
// 	cv::Mat_<double> camera_essential_mat;
// 	cv::Mat_<double> camera_fundamental_mat;
// };

// typedef point2i aruco_to_grid_idx[4];
// struct aruco_to_grid_idx{}

// Global objects - these shouldn't be here but they are so easy

struct calib_obj
{
	struct t_stereo_camera_calibration *calib = nullptr;
	struct xrt::auxiliary::tracking::StereoCameraCalibrationWrapper *our_wrap = nullptr;
	// struct StereoCameraCalibrationWrapper our_wrap;

	xrt_vec2 uv_lut[21][19];

	cv::Ptr<cv::aruco::Dictionary> our_dict;
	struct CaptureGrid grids[2][16 * 4];

	cv::Mat R[2];

	cv::Mat P1;
	cv::Mat P2;
	cv::Mat Q;
};



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

		if (id > corner_to_grid_pt.size()) {
			U_LOG_E("This probably shouldn't happen! id is bigger than array!");
			continue;
		}

		if (corner_to_grid_pt[id].size() == 0) {
			U_LOG_E("This probably shouldn't happen!");
			continue;
		}

		std::vector<point2i> map = corner_to_grid_pt[id];



		for (int j = 0; j < 4; j++) {
			int x_pt = map[j].x;
			int y_pt = map[j].y;
			char bob[64];
			sprintf(bob, "(%d, %d)", x_pt, y_pt);
			cv::putText(img, bob, all_corners[i][j], cv::FONT_HERSHEY_SIMPLEX, .3, {0, 0, 255}, 1);
		}
	}
}

struct xrt_vec2
raycoord(cv::Matx33d K, cv::Mat D, cv::Matx33d R, struct xrt_vec2 px_coord)
{
	cv::Mat in_px_coords(1, 1, CV_32FC2);
	float *write_in;
	write_in = in_px_coords.ptr<float>(0);
	write_in[0] = px_coord.x;
	write_in[1] = px_coord.y;
	cv::Mat out_ray(1, 1, CV_32FC2);

	cv::fisheye::undistortPoints(in_px_coords, out_ray, K, D);


	float n_x = out_ray.at<float>(0, 0);
	float n_y = out_ray.at<float>(0, 1);


	struct xrt_vec3 n = {n_x, n_y, 1.0f};

	struct xrt_vec3 o = {
	    static_cast<float>((n.x * R(0, 0)) + (n.y * R(0, 1)) + (n.z * R(0, 2))),
	    static_cast<float>((n.x * R(1, 0)) + (n.y * R(1, 1)) + (n.z * R(1, 2))),
	    static_cast<float>((n.x * R(2, 0)) + (n.y * R(2, 1)) + (n.z * R(2, 2))),
	};

	float amt = 1.0f / o.z;

	o.x *= amt;
	o.y *= amt;
	o.z *= amt;

	return {o.x, o.y};
}

void
process_into_capturegrid(calib_obj *obj,
                         int view,
                         std::vector<std::vector<cv::Point2f>> all_corners,
                         const std::vector<int> ids,
                         const std::vector<std::vector<point2i>> corner_to_grid_pt,
                         CaptureGrid *grid)
{
#if 0
	for (int x = 0; x < 19; x++) {
		for (int y = 0; y < 21; y++) {
			printf("%d %f %f\n", grid->corners[y][x].status, grid->corners[y][x].px_coord.x,
			       grid->corners[y][x].px_coord.y);
		}
	}
	printf("\n\n\n\n");
#endif


	for (int i = 0; i < corner_to_grid_pt.size(); i++) {
		for (int c = 0; c < 4; c++) {
			point2i blah = corner_to_grid_pt[i][c];
			// grid->corners[blah.y][blah.x].status = IN_PATTERN_NOT_CAPTURED;
			grid->corners[blah.y][blah.x].px_coord.x = 0;
			grid->corners[blah.y][blah.x].px_coord.y = 0;
		}
	}

	assert(all_corners.size() == ids.size());
	for (int a_idx = 0; a_idx < all_corners.size(); a_idx++) {
		int id = ids[a_idx];
		for (int c_idx = 0; c_idx < 4; c_idx++) {
			int x = corner_to_grid_pt[id][c_idx].x;
			int y = corner_to_grid_pt[id][c_idx].y;
			Corner *corner = &grid->corners[y][x];
			corner->status = CAPTURED;
			corner->px_coord.x = all_corners[a_idx][c_idx].x;
			corner->px_coord.y = all_corners[a_idx][c_idx].y;

			corner->bearing =
			    raycoord(obj->our_wrap->view[view].intrinsics_mat,
			             cv::Mat(obj->our_wrap->view[view].distortion_mat), obj->R[view], corner->px_coord);
		}
	}
}

int
main()
{
	calib_obj *obj = new calib_obj;

	load_basalt_calibration("/4/epics/AWE/calibrate_ns/calibrate_calibration_camera_2/result/calibration.json",
	                        &obj->calib);

	obj->our_wrap = new xrt::auxiliary::tracking::StereoCameraCalibrationWrapper(obj->calib);

	cv::fisheye::stereoRectify(obj->our_wrap->view[0].intrinsics_mat, //
	                           obj->our_wrap->view[0].distortion_mat, //
	                           obj->our_wrap->view[1].intrinsics_mat, //
	                           obj->our_wrap->view[1].distortion_mat, //
	                           cv::Size{1280, 800},                   //

	                           obj->our_wrap->camera_rotation_mat,    //
	                           obj->our_wrap->camera_translation_mat, //
	                           obj->R[0],                             //
	                           obj->R[1],                             //
	                           obj->P1,                               //
	                           obj->P2,                               //
	                           obj->Q,                                //
	                           0);

	std::cout << obj->R[0] << std::endl << obj->R[1] << std::endl;



	cv::Ptr<cv::aruco::Dictionary> our_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000);

	std::vector<std::vector<int>> offsets_list = {
	    {0, 0}, //
	    {1, 0}, //
	    {0, 1}, //
	    {1, 1}  //
	};

	bool first = true;

	int cap_ax = 0;
	while (true) {

		std::vector<int> offsets = offsets_list[cap_ax];
		cap_ax++;
		if (cap_ax > 3) {
			break;
			cap_ax = 0;
		}

		cv::Mat our_mat(1600, 1440, CV_8UC1);

		std::vector<std::vector<point2i>> grid_pts =
		    draw_arucos_2(offsets[0], offsets[1], our_mat(cv::Rect(0, 0, 1440, 1600)), our_dict);

		for (int view = 0; view < 2; view++) {
			for (int i = 0; i < 16; i++) {
				char name[1288];
				const char *side_name = view ? "right" : "left";
				sprintf(name, "captures/%d%d_%d_%s.png", offsets[0], offsets[1], i, side_name);
				U_LOG_E("%s", name);
				cv::Mat left = cv::imread(name, cv::IMREAD_GRAYSCALE);

				// cv::imshow("left", left);
				// cv::waitKey(0);
				std::vector<std::vector<cv::Point2f>> corners = {};
				std::vector<int> ids = {};

				cv::aruco::detectMarkers(left, our_dict, corners, ids);

				for (size_t i = 0; i < corners.size(); i++) {
					// 9*8 markers - 71st id is the last one. If we get something more than 71, it's
					// a mis-detection and we need to get rid of it (this has happened)
					//
					// It sucks that we're getting marker mis-detections but shouldn't be too common
					// fingers crossed.
					if (ids[i] > 71) {
						U_LOG_E("Erasing id %d in place %d!", ids[i], i);
						corners.erase(corners.begin() + i);
						ids.erase(ids.begin() + i);
					}
				}


				cv::Mat left_rgb = {};
				cv::cvtColor(left, left_rgb, cv::COLOR_GRAY2BGR);
				cv::aruco::drawDetectedMarkers(left_rgb, corners, ids);
				cv::imshow("hi", left_rgb);
				cv::waitKey(1);

				float sf = 1.8f;
				cv::resize(left, left, cv::Size(), sf, sf);
				cv::cvtColor(left, left, cv::COLOR_GRAY2BGR);

				std::vector<std::vector<cv::Point2f>> corners_2x;
				for (std::vector<cv::Point2f> corner_set : corners) {
					std::vector<cv::Point2f> new_corner_set;
					for (cv::Point2f &corner : corner_set) {
						new_corner_set.push_back({corner.x * sf, corner.y * sf});
					}
					corners_2x.push_back(new_corner_set);
				}

				debug_aruco_corners(left, corners_2x, ids, grid_pts);

				process_into_capturegrid(obj, view, corners, ids, grid_pts, &obj->grids[view][i]);
			}
		}
	}



	CaptureGrid both[2];


	for (int view = 0; view < 2; view++) {

		if (view == 0) {
			printf("\n\n\nLEFT:\n");
		} else {
			printf("\n\n\nRIGHT:\n");
		}

		for (int y = 0; y < 21; y++) {
			for (int x = 0; x < 19; x++) {
				printf("(% 05d % 05d) ", x, y);
			}
			printf("\n");

			for (int x = 0; x < 19; x++) {
				int num_captured = 0;
				float x_n = 0.0f;
				float y_n = 0.0f;
				bool cell_exists = false;


				for (int i = 0; i < 16 * 4; i++) {
					Corner *impor = &obj->grids[view][i].corners[y][x];
					if (impor->status == CAPTURED) {
						cell_exists = cell_exists || true;
						num_captured++;
						// printf("%f\n", impor->bearing.x);
						x_n += impor->bearing.x;
						y_n += impor->bearing.y;
					}
				}
				x_n /= num_captured;
				y_n /= num_captured;
				if (cell_exists) {
					printf("(% .2f % .2f) ", x_n, y_n);
					both[view].corners[y][x].bearing.x = x_n;
					both[view].corners[y][x].bearing.y = y_n;
					both[view].corners[y][x].status = CAPTURED;
				} else {
					both[view].corners[y][x].status = NOT_CAPTURED;

					both[view].corners[y][x].bearing.x = (float)x / (100.0f);
					both[view].corners[y][x].bearing.y = (float)-y / (100.0f);
					printf("nnnnnnnnnnnnn ");
				}
			}
			printf("\n\n");
		}
		fdg_run(&both[view]);
	}

	cJSON *root = cJSON_CreateObject();

	cJSON_AddStringToObject(root, "type", "Moshi's meshgrid-based distortion correction");
	cJSON_AddNumberToObject(root, "version", 2);

	cJSON_AddNumberToObject(root, "num_grid_points_x", 19);
	cJSON_AddNumberToObject(root, "num_grid_points_y", 21);

	cJSON_AddStringToObject(root, "desc",
	                        "for each member of `grids->{left, right}`, `grid` is a mapping from UV to bearing "
	                        "coordinates - rows are Y-axis, columns "
	                        "are X-axis, (0,0) UV is top-left of the screen and top-left of this array.");



	cJSON_AddNumberToObject(root, "ipd", 64.0);



	for (int view = 0; view < 2; view++) {

		cJSON *view_json = cJSON_AddObjectToObject(root, view ? "right" : "left");
		cJSON_AddNumberToObject(view_json, "dist_from_center", view ? .032f : -.032f);

		cJSON *grid = cJSON_AddArrayToObject(view_json, "grid");



		for (int y = 0; y < 21; y++) {

			cJSON *row = cJSON_CreateArray();
			cJSON_AddItemToArray(grid, row);

			for (int x = 0; x < 19; x++) {
				cJSON *cell = cJSON_CreateArray();
				cJSON *x_json = cJSON_CreateNumber(both[view].corners[y][x].bearing.x);
				cJSON *y_json = cJSON_CreateNumber(both[view].corners[y][x].bearing.y);
				cJSON_AddItemToArray(cell, x_json);
				cJSON_AddItemToArray(cell, y_json);
				cJSON_AddItemToArray(row, cell);
			}
		}
	}

	char *str = NULL;
	str = cJSON_Print(root);

	printf("%s\n", cJSON_Print(root));

	FILE *out_file = fopen("NSConfig_without_laplace.json", "w+");
	fprintf(out_file, "%s\n", str);
	fflush(out_file);
	fclose(out_file);
	out_file = NULL;
	free(str);


	cJSON_free(root);



	return 0;
}
