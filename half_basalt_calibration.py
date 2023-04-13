import json

# Load the original data from the json file
with open('/4/epics/AWE/calibrate_ns/ONE/basalt_calibration.json', 'r') as file:
    data = json.load(file)

print(data)

for camera in range(2):
    for name in "fx", "fy", "cx", "cy":
        data["value0"]["intrinsics"][camera]["intrinsics"][name] *= 0.5

with open('/4/epics/AWE/calibrate_ns/ONE/half_size/basalt_calibration.json', 'w+') as file:
    json.dump(data, file)

