#pragma once
#include <iostream>

#include "depthai/depthai.hpp"

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>


#include "cjson/cJSON.h"
#include "math/m_vec2.h"


// Structs

#define grid_width 19
#define grid_height 21

struct point2i
{
	int x;
	int y;
};

enum fdg_corner_status
{
	NOT_CAPTURED,
	CAPTURED,
};

struct Corner
{
	enum fdg_corner_status status;
	xrt_vec2 bearing = {0, 0};
  xrt_vec2 px_coord;
	// xrt_vec2 directions[8];
	// int num_directions;

	xrt_vec2 accum_force;
	float target_radius;

	Corner *straight_neighbors[4];
	int num_straight_neighbors = 0;

	Corner *diagonal_neighbors[4];
	int num_diagonal_neighbors = 0;

	// Bearing_Z is always 1.0f - these vectors are NOT normalized and can only
	// point in front of the view plane for now
};

struct CaptureGrid
{
	struct Corner corners[grid_height][grid_width];
};

struct WriteOutGrid {
  struct Corner corners[grid_height*4 - 3][grid_width*4 -3];
};
