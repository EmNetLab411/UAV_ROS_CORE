# UAV_ROS_CORE
This is ros package for PX4 autoplot with remote control, offboard mode, Slam autonomous...

## Installation

```bash
git clone git@github.com:EmNetLab411/UAV_ROS_CORE.git
cd UAV_ROS_CORE
catkin_make
source devel/setup.bash
```
## Usage

```bash
roslaunch uavlab411 uavlink.launch
```
# Update

### 27/09/2025

- Handle sending control signals from VC App (MODE: Manual, OffBoard)

### 28/09/2025

- Send Drone status to VC App:
  - Battery percent
  - Altitude
  - Latitude
  - Longitude
  - Velocity (local, GPS)
  - Orientation

### 30/09/2025

- Build function handle receive mission control from VC App
  - Circle Flight: implement in drone, receive signal control from VCApp
- Update send local position (home setup at position takeoff point)
