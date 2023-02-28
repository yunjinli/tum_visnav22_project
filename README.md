## Visual-Inertial Tracking using Preintegrated Factors

This project is built upon part of the code in practical course "Vision-based Navigation" (IN2106) taught at the Technical University of Munich.

## Setup

```
git clone --recursive https://github.com/yunjinli/tum_visnav22_project.git
```

```
cd tum_visnav22_project
./install_dependencies.sh
./build_submodules.sh
```

```
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
```

## Run

### Run Visual-Inertial Odometry (Ours)

```
cd build
./odometry --dataset-path ../data/euro_data/MH_05_difficult/mav0/ --cam-calib ../euroc_ds_calib_visnav_type.json --use-imu true
```

### Run Visual Odometry (Baseline)

```
cd build
./odometry --dataset-path ../data/euro_data/MH_05_difficult/mav0/ --cam-calib ../euroc_ds_calib_visnav_type.json --use-imu false
```

## Demo

### Machine Hall 04 - VO vs. VIO

[![MH04](https://img.youtube.com/vi/aNgcuXywrX4/0.jpg)](https://youtu.be/aNgcuXywrX4)

### Machine Hall 05 - VO vs. VIO

[![MH05](https://img.youtube.com/vi/fA9gDHygKfg/0.jpg)](https://youtu.be/fA9gDHygKfg)
