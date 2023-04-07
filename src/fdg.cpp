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

float repel_force;

void
make_neighbors(CaptureGrid *obj, int u, int v)
{
	Corner *us = &obj->corners[v][u];
	// Right
	if (u != grid_width - 1) {
		us->straight_neighbors[us->num_straight_neighbors++] = &obj->corners[v][u + 1];
	}
	// Left
	if (u != 0) {
		us->straight_neighbors[us->num_straight_neighbors++] = &obj->corners[v][u - 1];
	}

	// Down
	if (v != grid_height - 1) {
		us->straight_neighbors[us->num_straight_neighbors++] = &obj->corners[v + 1][u];
	}

	// Up
	if (v != 0) {
		us->straight_neighbors[us->num_straight_neighbors++] = &obj->corners[v - 1][u];
	}

	// Down-Right
	if ((u != grid_width - 1) && (v != grid_height - 1)) {
		us->diagonal_neighbors[us->num_diagonal_neighbors++] = &obj->corners[v + 1][u + 1];
	}

	// Up-Left
	if ((u != 0) && (v != 0)) {
		us->diagonal_neighbors[us->num_diagonal_neighbors++] = &obj->corners[v - 1][u - 1];
	}

	// Up-Right
	if ((u != grid_width - 1) && (v != 0)) {
		us->diagonal_neighbors[us->num_diagonal_neighbors++] = &obj->corners[v - 1][u + 1];
	}

	// Down-Left
	if ((u != 0) && (v != grid_height - 1)) {
		us->diagonal_neighbors[us->num_diagonal_neighbors++] = &obj->corners[v + 1][u - 1];
	}
}

void
calc_force_repulsion(CaptureGrid *obj, float amt, int our_u, int our_v)
{
	Corner *our = &obj->corners[our_v][our_u];
	for (int v = 0; v < grid_height; v++) {
		for (int u = 0; u < grid_width; u++) {
			if (v == our_v && u == our_u) {
				continue;
			}
			Corner *other = &obj->corners[v][u];
			// From other to us - we want to go away from other, always.
			xrt_vec2 dir = our->bearing - other->bearing;
			float mag = amt / m_vec2_len(dir);

			m_vec2_normalize(&dir);

			our->accum_force += m_vec2_mul_scalar(dir, mag);
			if (our->accum_force.x != our->accum_force.x) {
				printf("uh oh: zero length\n");
				our->accum_force = {0, 0};
			}
			// printf("%f %f\n", our->accum_force.x, our->accum_force.y);
		}
	}
}

float
calc_avg_neighbor_len(CaptureGrid *obj, int u, int v)
{
	float avg_len = 0;
	int num_edges = 0;
	xrt_vec2 us = obj->corners[v][u].bearing;
	for (int i = 0; i < obj->corners[v][u].num_diagonal_neighbors; i++) {
		avg_len += m_vec2_len(us - obj->corners[v][u].diagonal_neighbors[i]->bearing) / (1.414);
		num_edges += 1;
	}

	for (int i = 0; i < obj->corners[v][u].num_straight_neighbors; i++) {
		avg_len += m_vec2_len(us - obj->corners[v][u].straight_neighbors[i]->bearing);
		num_edges += 1;
	}
	avg_len /= num_edges;
	return avg_len;
}

void
calc_force_neighbor_correct_length(CaptureGrid *obj, float desired_length, int u, int v)
{
	xrt_vec2 us = obj->corners[v][u].bearing;

	// This part works. I'm not sure what b I should select.
	// for (int i = 0; i < obj->corners[v][u].num_diagonal_neighbors; i++) {
	// 	// From us to them
	// 	xrt_vec2 dir = obj->corners[v][u].diagonal_neighbors[i]->bearing - us;
	// 	float go_amount = m_vec2_len(dir)
	// 	obj->corners[v][u].accum_force += m_vec2_mul_scalar(dir, b / (1.414*6));
	// }

	for (int i = 0; i < obj->corners[v][u].num_diagonal_neighbors; i++) {
		// From us to them
		xrt_vec2 dir = obj->corners[v][u].diagonal_neighbors[i]->bearing - us;
		float go_amount = m_vec2_len(dir) - desired_length*1.414;
		obj->corners[v][u].accum_force += m_vec2_mul_scalar(dir, go_amount*.32);
	}

	for (int i = 0; i < obj->corners[v][u].num_straight_neighbors; i++) {
		// From us to them
		xrt_vec2 dir = obj->corners[v][u].straight_neighbors[i]->bearing - us;
		float go_amount = m_vec2_len(dir) - desired_length;
		obj->corners[v][u].accum_force += m_vec2_mul_scalar(dir, go_amount);
	}
}

void
calc_forces(CaptureGrid *obj)
{
	for (int v = 0; v < grid_height; v++) {
		for (int u = 0; u < grid_width; u++) {
			if (obj->corners[v][u].status == CAPTURED) {
				continue;
			}
			obj->corners[v][u].accum_force = {0, 0};
			calc_force_repulsion(obj, repel_force, u, v);
			float len = calc_avg_neighbor_len(obj, u, v);
			calc_force_neighbor_correct_length(obj, len * .95, u, v);
		}
	}
}

static void
apply_forces(CaptureGrid *obj)
{
	for (int v = 0; v < grid_height; v++) {
		for (int u = 0; u < grid_width; u++) {
			if (obj->corners[v][u].status == CAPTURED) {
				continue;
			}
			obj->corners[v][u].bearing += m_vec2_mul_scalar(obj->corners[v][u].accum_force, 1.0);
		}
	}
}

static cv::Point2f
bear2px(xrt_vec2 bear)
{
	bear.x *= 720;
	bear.y *= 720;
	bear.x += 720;
	bear.y += 720;

	return {bear.x, bear.y};
}
//26cb7c
static void
display_grid(CaptureGrid *obj)
{
	cv::Mat hey(1440, 1440, CV_8UC3);
	// right, right-down, down, down-left
	cv::Scalar color = {0x2b, 0xcb, 0x7c};
	for (int v = 0; v < grid_height; v++) {
		for (int u = 0; u < grid_width; u++) {
			// Right
			if (u != grid_width - 1) {
				cv::line(hey, bear2px(obj->corners[v][u].bearing),
				         bear2px(obj->corners[v][u + 1].bearing), color);
			}
			// down-right
			if (v != grid_height - 1 && u != grid_width - 1) {
				cv::line(hey, bear2px(obj->corners[v][u].bearing),
				         bear2px(obj->corners[v + 1][u + 1].bearing), color);
			}
			// down
			if (v != grid_height - 1) {
				cv::line(hey, bear2px(obj->corners[v][u].bearing),
				         bear2px(obj->corners[v + 1][u].bearing), color);
			}
			// down-left
			if (v != grid_height - 1 && u != 0) {
				cv::line(hey, bear2px(obj->corners[v][u].bearing),
				         bear2px(obj->corners[v + 1][u - 1].bearing), color);
			}
		}
	}
	cv::imshow("graph!", hey);
	cv::waitKey(1);
}

void
fdg_run(CaptureGrid *obj)
{
	for (int v = 0; v < grid_height; v++) {
		for (int u = 0; u < grid_width; u++) {
			make_neighbors(obj, u, v);
		}
	}
	display_grid(obj);

	repel_force =  0.000003;

	int num_iterations = 3000;
	for (int i = 0; i < num_iterations; i++) {
		if (i > num_iterations / 2) {
			repel_force =  0.000001;
		}
		// printf("\n");
		printf("%d / %d\r", i, num_iterations);
		calc_forces(obj);
		apply_forces(obj);
		display_grid(obj);
	}
	printf("\n");
}