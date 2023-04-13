#include <stdio.h>
#include "load_basalt_calibration.hpp"

int
main()
{
	t_stereo_camera_calibration *calib = NULL;
	load_basalt_calibration("/4/epics/AWE/calibrate_ns/ONE/basalt_calibrate/result/calibration_no_imu.json", &calib);

	t_stereo_camera_calibration_save("meow.json", calib);

	system(
	    "python3 /4/epics/AWE/calibrate_ns/calibrate-hmd-v3/monado/scripts/upload_calibration_to_depthai.py "
	    " --baseline 7.5 /4/epics/AWE/calibrate_ns/calibrate-hmd-v3/build/meow.json ");

	printf("hi\n");
}