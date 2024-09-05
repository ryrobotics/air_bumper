# Air Bumper

Source code for the paper: **Air Bumper: A Collision Detection and Reaction Framework for Autonomous MAV Navigation**. For details on the mapping modules used within Air Bumper, please refer to [Collision-Aware-GIE-Mapping](https://github.com/ryrobotics/Collision-Aware-GIE-Mapping).

This work has been accepted by 2024 IEEE International Conference on Robotics and Automation (ICRA).


Air Bumper is a collision detection, estimation and reaction framework, which enables MAVs to recover from collisions with unobserved obstacles, like glass doors, and unpredictable hits. The proposed framework achieves real-time on-board collision detection and reactions in both simulated environments and real-world scenarios. 

## Demo Video

<p align="center">
  <a href="https://youtu.be/FVQGmqUTyp4" target="_blank"><img src="https://github.com/ryrobotics/ryrobotics.github.io/blob/main/content/publication/wang-2023-air/featured.png" alt="video" width="800" height="450" border="1" /></a>
</p>

## Citation
If this project aids your research, please consider citing our paper:

```
@inproceedings{wang2024air,
  title={Air Bumper: A Collision Detection and Reaction Framework for Autonomous MAV Navigation},
  author={Wang, Ruoyu and Guo, Zixuan and Chen, Yizhou and Wang, Xinyi and Chen, Ben M},
  booktitle={2024 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={15735--15741},
  year={2024},
  organization={IEEE}}
```
Don't forget to star :star: this repository if it's helpful to your projects!

## 1. Installation

### Prerequisites

- **Ubuntu with ROS:** This project has been tested on Ubuntu 18.04 (ROS Melodic) and 20.04 (ROS Noetic). For ease of installation, we recommend performing a Desktop-Full Install of ROS.
    ```bash
    sudo apt install ros-${ROS_DISTRO}-desktop-full
    ```
    For ARM64 platforms, the ROS Perception package can simplify installation.
    ```bash
    sudo apt install ros-${ROS_DISTRO}-perception
    ```

- **Nvidia GPU (Optional):** Required only for [Collision-Aware-GIE-Mapping](https://github.com/ryrobotics/Collision-Aware-GIE-Mapping) and **gradient-based collision recovery control** features.

- **MAVROS Installation:**
    ```bash
    sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs

    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh

    sudo bash ./install_geographiclib_datasets.sh   
    ```

### Building on ROS
```bash
mkdir -p catkin_ws/src && cd catkin_ws/src
git clone https://github.com/ryrobotics/air_bumper.git
cd ..
catkin build
source devel/setup.bash
```

### Optional: Mapping Modules Installation

Follow the instructions in [Collision-Aware-GIE-Mapping](https://github.com/ryrobotics/Collision-Aware-GIE-Mapping) if you require **collision-aware map**  or **gradient-based collision recovery control**.


## 2. Integrate with PX4 and Motion Planning

### Integrate with PX4

Integrating Air Bumper with PX4 involves utilizing a custom PX4 controller module to manage control commands and relay ROS messages to the flight controller. [Prometheus](https://github.com/amov-lab/Prometheus) and [px4ctrl](https://github.com/ZJU-FAST-Lab/Fast-Drone-250/tree/master/src/realflight_modules/px4ctrl) may be a good start.

We provide two custom messages for integration:

- **activity_pose.msg**: Contains the target position and type. The **type** of activity_pose should be interpreted by the controller to determine if the command is for an emergency collision-reaction.
- **confirmation.msg**: Used to provide feedback from the controller to Air Bumper, assisting in determining if the MAV has reached the target position.

### Integrate with Motion Planning

Air Bumper seamlessly integrates with motion planning algorithms. The [Collision-Aware-GIE-Mapping](https://github.com/ryrobotics/Collision-Aware-GIE-Mapping) module within the framework publishes the EDT around the robot as `CostMap.msg` on the "cost_map" topic. Each voxel in the map includes visibility information and distance values.

Upon a collision, the **collision point cloud** is processed by the mapping modules, and the updated `cost_map` will contain the collision information. Motion planning algorithms simply need to subscribe to the "cost_map" topic. For further details, refer to the [instructions](https://github.com/ryrobotics/Collision-Aware-GIE-Mapping?tab=readme-ov-file#2-try-on-your-own-robot) in the Collision-Aware-GIE-Mapping repo.

## 3. Try It Using PX4 SITL

To facilitate the deployment of Air Bumper with PX4 SITL in Gazebo, we provide models, worlds, and launch files.

We recommend following the [PX4 development environment setup instructions](https://docs.px4.io/v1.13/en/dev_setup/dev_env_linux_ubuntu.html#bash-scripts) to prepare your system.

Move all provided files in `air_bumper/simulation/PX4-Autopilot/` to your **PX4-Autopilot** directory. The `px4_air_bumper.launch` file launches a simulation environment used in the paper,including a MAV with 3D LiDAR, two glass doors, and one standard door.

For simulating a MAV with 3D LiDAR, move the `air_bumper/simulation/velodyne_simulator` folder to your workspace (e.g., `catkin_ws`) and compile it. This plugin simulates a Velodyne LiDAR and the parameters can be adjusted in `3d_gpu_lidar.sdf`.

## 4. Try on Your Own Robot

Deploying Air Bumper on your robot follows steps similar to those for SITL, with the main difference being the handling of real-world sensors. You may need to remap topics and ensure your sensors are robust enough for collision scenarios.

We've provided several launch files and scripts to support your deployment. Launch files intended for simulation use the **_sitl** suffix to distinguish them from those used in real-world deployments. To simplify the start-up process of the entire framework, we recommend using *tmuxinator*.

## 5. License
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.
