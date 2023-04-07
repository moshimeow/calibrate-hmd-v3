#include <iostream>

#include "depthai/depthai.hpp"

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>


#include "cjson/cJSON.h"
#include "defines.hpp"
#include "fdg.hpp"

// Structs

struct Whatever
{
	cv::Size image_size_pixels_cv;
	cv::Matx<double, 3, 3> cameraMatrix;
	cv::Matx<double, 4, 1> distortion_fisheye_mat;
	bool use_fisheye;
};

struct StereoCameraCalibrationWrapper
{
	Whatever view[2];
	cv::Mat_<double> camera_translation_mat;
	cv::Matx<double, 3, 3> camera_rotation_mat;
	cv::Mat_<double> camera_essential_mat;
	cv::Mat_<double> camera_fundamental_mat;
};

// typedef point2i aruco_to_grid_idx[4];
// struct aruco_to_grid_idx{}

// Global objects - these shouldn't be here but they are so easy

struct calib_obj
{
	struct StereoCameraCalibrationWrapper our_wrap;

	xrt_vec2 uv_lut[21][19];

	cv::Ptr<cv::aruco::Dictionary> our_dict;
	struct CaptureGrid grids[2][16 * 4];

	cv::Mat R[2];

	cv::Mat P1;
	cv::Mat P2;
	cv::Mat Q;
};

calib_obj *the_one_and_only;

void
dummy_fill_wrap(calib_obj *obj)
{
	obj->our_wrap.view[0].image_size_pixels_cv.height = 800;
	obj->our_wrap.view[0].image_size_pixels_cv.width = 1280;

	obj->our_wrap.view[1].image_size_pixels_cv.height = 800;
	obj->our_wrap.view[1].image_size_pixels_cv.width = 1280;

	obj->our_wrap.view[0].cameraMatrix = {508.2431317803847, 0, 625.1262964163459,
	                                      //
	                                      0, 508.0092415823303, 436.0861794525318,
	                                      //
	                                      0, 0, 1}; // Shut up

	obj->our_wrap.view[1].cameraMatrix = {508.0612269474984, 0, 646.8775239309249,
	                                      //
	                                      0, 507.794410062113, 348.0291105458621,
	                                      //
	                                      0, 0, 1}; // Shhhhhh

	obj->our_wrap.view[0].distortion_fisheye_mat = {-0.01178353719932522, -0.002743341972778042,
	                                                5.13223456164071e-05, -6.336463736072942e-05};

	obj->our_wrap.view[1].distortion_fisheye_mat = {-0.01308899660105348, 0.004549237696403933,
	                                                -0.00751955708545033, 0.00240647121573423};

	obj->our_wrap.view[0].use_fisheye = true;
	obj->our_wrap.view[1].use_fisheye = true;

	obj->our_wrap.camera_rotation_mat = {0.99998401008472138,    -0.0056288582865031264, -0.00054362603901055831,
	                                     0.005632298495140923,   0.99996269403283633,    0.0065488744264765369,
	                                     0.00050674307243268977, -0.006551831574650723,  0.99997840812413341};
	obj->our_wrap.camera_translation_mat =
	    cv::Mat_<double>{-0.045219290732227996, -0.00011277813077547463, 6.1651923527060852e-05};

	cv::fisheye::stereoRectify(obj->our_wrap.view[0].cameraMatrix,           //
	                           obj->our_wrap.view[0].distortion_fisheye_mat, //
	                           obj->our_wrap.view[1].cameraMatrix,           //
	                           obj->our_wrap.view[1].distortion_fisheye_mat, //
	                           cv::Size{1280, 800},                          //

	                           obj->our_wrap.camera_rotation_mat,    //
	                           obj->our_wrap.camera_translation_mat, //
	                           obj->R[0],                            //
	                           obj->R[1],                            //
	                           obj->P1,                              //
	                           obj->P2,                              //
	                           obj->Q,                               //
	                           0);

	std::cout << obj->R[0] << std::endl << obj->R[1] << std::endl;
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

			corner->bearing = raycoord(obj->our_wrap.view[view].cameraMatrix,
			                           cv::Mat(obj->our_wrap.view[view].distortion_fisheye_mat),
			                           obj->R[view], corner->px_coord);
			// printf("%f %f\n", corner->bearing.x, corner->bearing.y);
		}
	}
}

int
main()
{
	the_one_and_only = new calib_obj;
	dummy_fill_wrap(the_one_and_only);

	cv::namedWindow("hi", 0);
	cv::moveWindow("hi", 2000, 0);
	cv::setWindowProperty("hi", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

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
				cv::Mat left = cv::imread(name, cv::IMREAD_GRAYSCALE);
				std::vector<std::vector<cv::Point2f>> corners;
				std::vector<int> ids;

				cv::aruco::detectMarkers(left, our_dict, corners, ids);

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

				process_into_capturegrid(the_one_and_only, view, corners, ids, grid_pts,
				                         &the_one_and_only->grids[view][i]);

				// cv::aruco::drawDetectedMarkers(left, corners_2x, ids);
				// cv::imshow("hi", left);
				// cv::waitKey(1);
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
					Corner *impor = &the_one_and_only->grids[view][i].corners[y][x];
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
					printf("nnnnnnnnnnnnn ");
				}
			}
			printf("\n\n");
		}
		fdg_run(&both[view]);
	}

	cJSON *root = cJSON_CreateObject();

	cJSON_AddStringToObject(root, "calibration-type", "Numba two babey!!!!!");

	cJSON_AddNumberToObject(root, "num_grid_points_x", 19);
	cJSON_AddNumberToObject(root, "num_grid_points_y", 21);

	cJSON_AddStringToObject(root, "desc",
	                        "for each member of `grids->{left, right}`, `grid` is a mapping from UV to bearing "
	                        "coordinates - rows are Y-axis, columns "
	                        "are X-axis, (0,0) UV is top-left of the screen and top-left of this array.");


	cJSON *grids_list = cJSON_AddArrayToObject(root, "grids");

	cJSON *jorge = cJSON_CreateObject();

	cJSON_AddItemToArray(grids_list, jorge);

	cJSON_AddNumberToObject(jorge, "ipd", .064);



	for (int view = 0; view < 2; view++) {

		cJSON *view_json = cJSON_AddObjectToObject(jorge, view ? "right" : "left");
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

	char *hi = NULL;
	hi = cJSON_Print(root);

	printf("%s\n", cJSON_Print(root));

	free(hi);

	cJSON_free(root);

#if 0

		float max_x_dist = 0.0f;
		float max_y_dist = 0.0f;

		for (int y = 0; y < 21; y++) {
			for (int x = 0; x < 19; x++) {

				float x_min = NAN;
				float y_min = NAN;

				float x_max = NAN;
				float y_max = NAN;

				for (int i = 0; i < 16 * 4; i++) {
					Corner *impor = &the_one_and_only->grids[view][i].corners[y][x];
					if (impor->status == CAPTURED) {
						// printf("%f %f\n", impor->px_coord.x, impor->px_coord.y);
						x_max = fmax(impor->px_coord.x, x_max);
						x_min = fmin(impor->px_coord.x, x_max);

						y_max = fmax(impor->px_coord.y, y_max);
						y_min = fmin(impor->px_coord.y, y_max);
					}
					max_x_dist = fmax(abs(x_max - x_min), max_x_dist);
					max_y_dist = fmax(abs(y_max - y_min), max_y_dist);
				}
				// printf("\n\n\n");
			}
		}
		printf("X wobble (px coords): %f \nY wobble (px coords): %f\n", max_x_dist, max_y_dist);
	}

#endif

	// cv::waitKey(0);

	return 0;
}
