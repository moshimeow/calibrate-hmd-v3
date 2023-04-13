#include <iostream>

#include "depthai/depthai.hpp"

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>


#include "cjson/cJSON.h"
#include "fdg.hpp"
#include "util/u_json.h"
#include "defines.hpp"

void
load_error()
{
	abort();
}

void
parse_error()
{
	abort();
}

CaptureGrid start_grids[2];

enum LapCornerType
{
	REAL,
	// FDG,
	INTER
};

struct LapCorner
{
	enum LapCornerType type;
	float acceptable_move_radius;
	xrt_vec2 orig_bearing;
	xrt_vec2 curr_bearing;
	xrt_vec2 next_bearing;
};

struct LapGrid
{
	int height = grid_height * 2 - 1;
	int width = grid_width * 2 - 1;
	struct LapCorner corners[grid_height * 2 - 1][grid_width * 2 - 1];
};


static cv::Point2f
bear2px(xrt_vec2 bear)
{
	bear.x *= 720 * 1.23;
	bear.y *= 720 * 1.23;
	bear.x += 700;
	bear.y += 700;

	return {bear.x, bear.y};
}


static void
lap_display_grid(LapGrid *obj, bool show_back, bool show_front)
{
	// Uninitialized here!
	cv::Mat hey(1400, 1400, CV_8UC3);

	// Initialize it by making sure all the pixels are 0
	hey = 0;
	// right, right-down, down, down-left

	if (show_back) {
		cv::Scalar color = {0x00, 0xcb, 0xfc};
		for (int v = 0; v < obj->height; v++) {
			for (int u = 0; u < obj->width; u++) {
				// Right
				if (u != obj->width - 1) {
					cv::line(hey, bear2px(obj->corners[v][u].orig_bearing),
					         bear2px(obj->corners[v][u + 1].orig_bearing), color);
				}
				// down
				if (v != obj->height - 1) {
					cv::line(hey, bear2px(obj->corners[v][u].orig_bearing),
					         bear2px(obj->corners[v + 1][u].orig_bearing), color);
				}
			}
		}
	}
	if (show_front) {
		cv::Scalar color = {0x2b, 0xcb, 0x7c};
		for (int v = 0; v < obj->height; v++) {
			for (int u = 0; u < obj->width; u++) {
				// Right
				if (u != obj->width - 1) {
					cv::line(hey, bear2px(obj->corners[v][u].curr_bearing),
					         bear2px(obj->corners[v][u + 1].curr_bearing), color);
				}
				// down
				if (v != obj->height - 1) {
					cv::line(hey, bear2px(obj->corners[v][u].curr_bearing),
					         bear2px(obj->corners[v + 1][u].curr_bearing), color);
				}
			}
		}
	}
	cv::imshow("graph!", hey);
}


int
main()
{
	const char *config_path = "NSConfig_without_laplace.json";
	FILE *config_file = fopen(config_path, "r");
	if (config_file == NULL) {
		load_error();
	}

	fseek(config_file, 0, SEEK_END);    // Go to end of file
	int file_size = ftell(config_file); // See offset we're at. This should be the file size in bytes.
	rewind(config_file);                // Go back to the beginning of the file

	if (file_size == 0) {
		load_error();
	} else if (file_size > 30 * pow(1024, 2)) { // 30 MiB
		load_error();
	}

	char *json = (char *)calloc(file_size + 1, 1);

	fread(json, 1, file_size, config_file);
	fclose(config_file);
	json[file_size] = '\0';

	cJSON *config_json = cJSON_Parse(json);



	// Assume height 21, width 19.

	// u_json_get_int(u_json_get(config_json, "num_grid_points_x"), &temp_data->num_grid_points_u);
	// u_json_get_int(u_json_get(config_json, "num_grid_points_y"), &temp_data->num_grid_points_v);

	// temp_data->grid[0] = realloc(temp_data->grid[0], sizeof(struct xrt_vec2) * temp_data->num_grid_points_u *
	//                                                      temp_data->num_grid_points_v);
	// temp_data->grid[1] = realloc(temp_data->grid[1], sizeof(struct xrt_vec2) * temp_data->num_grid_points_u *
	//                                                      temp_data->num_grid_points_v);

	const cJSON *current_element = NULL;



	for (int view = 0; view < 2; view++) {
		const struct cJSON *grid_root = u_json_get(config_json, view ? "right" : "left");
		grid_root = u_json_get(grid_root, "grid");
		// if view is 0, then left. if view is 1, then right
		for (int lv = 0; lv < grid_height; lv++) {
			struct cJSON *v_axis = cJSON_GetArrayItem(grid_root, lv);
			// printf("v:axis: %s\n", cJSON_Print(v_axis));

			for (int lu = 0; lu < grid_width; lu++) {
				struct cJSON *cell = cJSON_GetArrayItem(v_axis, lu);

				struct cJSON *cellX = cJSON_GetArrayItem(cell, 0);
				struct cJSON *cellY = cJSON_GetArrayItem(cell, 1);
				if (grid_root == NULL || cell == NULL || v_axis == NULL || cellX == NULL ||
				    cellY == NULL) {
					parse_error();
				}
				float *x_ptr = &start_grids[view].corners[lv][lu].bearing.x;
				float *y_ptr = &start_grids[view].corners[lv][lu].bearing.y;
				if (cJSON_IsNull(cellX) || cJSON_IsNull(cellY)) {
					*x_ptr = NAN;
					*y_ptr = NAN;
				}
				u_json_get_float(cellX, x_ptr);
				u_json_get_float(cellY, y_ptr);
				start_grids[view].corners[lv][lu].status = CAPTURED;
			}
			// printf("\n");
		}
	}


	// display_grid(&start_grids[0]);
	// cv::waitKey(0);

	LapGrid real_grids[2];
	for (int view = 0; view < 2; view++) {
		for (int v = 0; v < grid_height; v++) {
			for (int u = 0; u < grid_width; u++) {
				LapCorner *the = &real_grids[view].corners[v * 2][u * 2];
				the->orig_bearing = start_grids[view].corners[v][u].bearing;
				the->curr_bearing = the->orig_bearing;
				the->acceptable_move_radius = 0.001;

				if (u != grid_width - 1) {
					xrt_vec2 start = start_grids[view].corners[v][u].bearing;
					xrt_vec2 end = start_grids[view].corners[v][u + 1].bearing;
					the = &real_grids[view].corners[v * 2][u * 2 + 1];
					the->orig_bearing = m_vec2_lerp(start, end, 0.5f);
					the->curr_bearing = the->orig_bearing;
					the->acceptable_move_radius = 0.001;
				}
				if (v != grid_height - 1) {
					xrt_vec2 start = start_grids[view].corners[v][u].bearing;
					xrt_vec2 end = start_grids[view].corners[v + 1][u].bearing;
					the = &real_grids[view].corners[v * 2 + 1][u * 2];
					the->orig_bearing = m_vec2_lerp(start, end, 0.5f);
					the->curr_bearing = the->orig_bearing;
					the->acceptable_move_radius = 0.001;
				}
				if ((v != grid_height - 1) && (u != grid_width - 1)) {
					xrt_vec2 start_low_v = start_grids[view].corners[v][u].bearing;
					xrt_vec2 end_low_v = start_grids[view].corners[v][u + 1].bearing;

					xrt_vec2 start_high_v = start_grids[view].corners[v + 1][u].bearing;
					xrt_vec2 end_high_v = start_grids[view].corners[v + 1][u + 1].bearing;
					the = &real_grids[view].corners[v * 2 + 1][u * 2 + 1];
					the->orig_bearing = m_vec2_lerp(m_vec2_lerp(start_high_v, end_high_v, 0.5f),
					                                m_vec2_lerp(start_low_v, end_low_v, 0.5), 0.5f);

					the->curr_bearing = the->orig_bearing;
					the->acceptable_move_radius = 0.001;
				}
			}
		}
	}
	bool show_back = true;
	bool show_front = true;
	lap_display_grid(&real_grids[0], show_back, show_front);
	cv::waitKey(1);
	int view = 0;
	for (int view = 0; view < 2; view++) {
		for (int i = 0; i < 100; i++) {
			for (int v = 1; v < real_grids[0].height - 1; v++) {
				for (int u = 1; u < real_grids[0].width - 1; u++) {
					xrt_vec2 the_new = {0, 0};
					the_new += real_grids[view].corners[v][u - 1].curr_bearing;
					the_new += real_grids[view].corners[v][u + 1].curr_bearing;
					the_new += real_grids[view].corners[v - 1][u].curr_bearing;
					the_new += real_grids[view].corners[v + 1][u].curr_bearing;

					the_new = m_vec2_mul_scalar(the_new, .25);
					xrt_vec2 from_old_to_us = the_new - real_grids[view].corners[v][u].orig_bearing;

					if (m_vec2_len(from_old_to_us) >
					    real_grids[view].corners[v][u].acceptable_move_radius) {
						from_old_to_us = m_vec2_mul_scalar(
						    from_old_to_us,
						    real_grids[view].corners[v][u].acceptable_move_radius /
						        m_vec2_len(from_old_to_us));
					}

					real_grids[view].corners[v][u].next_bearing =
					    real_grids[view].corners[v][u].orig_bearing + from_old_to_us;


					// real_grids[view].corners[v][u].
				}
			}
			for (int v = 1; v < real_grids[0].height - 1; v++) {
				for (int u = 1; u < real_grids[0].width - 1; u++) {
					real_grids[view].corners[v][u].curr_bearing =
					    real_grids[view].corners[v][u].next_bearing;
				}
			}
			lap_display_grid(&real_grids[0], show_back, show_front);
			char key = cv::waitKey(1);
#if 0
		if (key == 'f') {
			show_front = !show_front;
			lap_display_grid(&real_grids[0], show_back, show_front);
			// cv::waitKey(0);
		}
		if (key == 'b') {
			show_back = !show_back;
			lap_display_grid(&real_grids[0], show_back, show_front);
			// cv::waitKey(0);
		}
#endif
		}
	}

	cv::waitKey(0);

	cJSON *root = cJSON_CreateObject();

	cJSON_AddStringToObject(root, "type", "meshgrid");
	cJSON_AddNumberToObject(root, "version", 2);

	cJSON_AddNumberToObject(root, "num_grid_points_x", real_grids[0].width);
	cJSON_AddNumberToObject(root, "num_grid_points_y", real_grids[0].height);

	cJSON_AddStringToObject(root, "desc",
	                        "for each member of `grids->{left, right}`, `grid` is a mapping from UV to bearing "
	                        "coordinates - rows are Y-axis, columns "
	                        "are X-axis, (0,0) UV is top-left of the screen and top-left of this array.");



	cJSON_AddNumberToObject(root, "ipd", 64.0);



	for (int view = 0; view < 2; view++) {

		cJSON *view_json = cJSON_AddObjectToObject(root, view ? "right" : "left");
		cJSON_AddNumberToObject(view_json, "dist_from_center", view ? .032f : -.032f);

		cJSON *grid = cJSON_AddArrayToObject(view_json, "grid");



		for (int y = 0; y < real_grids[0].height; y++) {

			cJSON *row = cJSON_CreateArray();
			cJSON_AddItemToArray(grid, row);

			for (int x = 0; x < real_grids[0].width; x++) {
				cJSON *cell = cJSON_CreateArray();
				cJSON *x_json = cJSON_CreateNumber(real_grids[view].corners[y][x].curr_bearing.x);
				cJSON *y_json = cJSON_CreateNumber(real_grids[view].corners[y][x].curr_bearing.y);
				cJSON_AddItemToArray(cell, x_json);
				cJSON_AddItemToArray(cell, y_json);
				cJSON_AddItemToArray(row, cell);
			}
		}
	}

	char *str = NULL;
	str = cJSON_Print(root);

	printf("%s\n", cJSON_Print(root));

	FILE *out_file = fopen("NSConfig.json", "w+");
	fprintf(out_file, "%s\n", str);
	fflush(out_file);
	fclose(out_file);
	out_file = NULL;
	free(str);


	cJSON_free(root);



	return 0;
}
