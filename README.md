# k4a_extract_data
This repository contains C++ code to extract depth and color images from Azure Kinect .mkv recordings using the Azure Kinect Sensor SDK.

The code is available for non-commercial research purposes under [CC BY-NC 4.0](https://creativecommons.org/licenses/by-nc/4.0/) license.

### Dependencies

- Azure Kinect Sensor SDK
- OpenCV
- CMake

### Installation
```
mkdir build
cd build
cmake ..
make
```

This code has been tested on Ubuntu 20.04, Azure Kinect SDK 1.4.1 and OpenCV 4.7.0

### Running

```
./bin/k4a_extract_data <mkv_filename> [<rotation angle (90, 180, 270), default: 0>] [<from_TS> <to_TS> <task_name>]
```
`rotation angle`: optional, to rotate extracted images (e.g., if the camera was rotated during recording)

`fromTS`, `toTS`: optional, to extract only a portion of the recording, starting at timestamp fromTS until toTS (in microseconds)

`task_name`: string that will be appended to the output folder

Example:
`./bin/k4a_extract_data ~/test/test123.mkv 90 0 1000000 part1` will store images rotated by 90 degrees for the first second (1000000 microsec) of recording `test123.mkv` in folder  `~/test_part1`
