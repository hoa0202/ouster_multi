# ouster_multi

Dual Ouster LiDAR driver for ROS2. Launches front and rear OS-0 sensors simultaneously with separate namespaces.

## Setup

```bash
mkdir -p ~/ouster_ws/src && cd ~/ouster_ws/src
git clone --recursive https://github.com/hoa0202/ouster_multi.git
cd ~/ouster_ws
rosdep install --from-paths src -y --ignore-src
colcon build
source install/setup.bash
```

## Usage

```bash
# Default (full 360°)
ros2 launch ouster_multi dual_driver.launch.py

# With azimuth window (millidegrees, 0-360000)
ros2 launch ouster_multi dual_driver.launch.py \
  front_azimuth_start:=270000 front_azimuth_end:=90000 \
  rear_azimuth_start:=270000 rear_azimuth_end:=90000
```

## Sensors

| Name | IP | Namespace | Topics |
|------|------|-----------|--------|
| Front | 192.168.0.93 | `/ouster_front` | `/ouster_front/points`, `/ouster_front/imu`, ... |
| Rear | 192.168.0.24 | `/ouster_rear` | `/ouster_rear/points`, `/ouster_rear/imu`, ... |

## Config

Edit `config/driver_params_front.yaml` and `config/driver_params_rear.yaml` to change sensor IP, frame names, proc_mask, etc.
