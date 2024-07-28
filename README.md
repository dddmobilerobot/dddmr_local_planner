# dddmr_local_planner

DDDmobileRobot local planner including following plugined-based features:
- Trajectory generator
- Critics
- Recovery behaviors

Our local planner is similar as Nav2 DWB planner but differentiate from following perspectives:
- Our local planner is for 3D navigation, the collision and trajectory rating are computed in 3D perspective. For example, the collision check in 2D is by checking a point in a polygon, however, in 3D we check collision by checking a point cloud in a cuboid.
- Our local planner enable each trajectory generator to have seperated critics and weights, which allowing users to setup flexibility behaviors for the robot.
<p align='center'>
    <img src="https://github.com/dddmobilerobot/dddmr_documentation_materials/blob/main/local_planner/local_planner_play_ground_annotated.png" width="720" height="420"/>
</p>


## Run The Demo
### 1. Create docker image
The package runs in the docker, so we need to build the image first. We support both x64 (tested in intel NUC) and arm64 (tested in nvidia jetson jpack6).
```
cd ~
git clone https://github.com/dddmobilerobot/dddmr_navigation.git
cd ~/dddmr_navigation && git submodule init && git submodule update
cd ~/dddmr_navigation/dddmr_docker/docker_file && ./build.bash
```
### 2. Run demo
#### Create a docker container
> [!NOTE]
> The following command will create an interactive docker container using the image we built. The we can launch the demo in the container.
```
cd ~/dddmr_navigation/dddmr_docker && ./run_demo.bash
```
#### Launch local planner play ground
```
cd ~/dddmr_navigation && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch local_planner local_planner_play_ground.launch
```
#### Use Publish Point on Rviz2
Use Publish Point to give a goal to the local planner, and it will generate a prune plan to the goal you provide.
<p align='center'>
    <img src="https://github.com/dddmobilerobot/dddmr_documentation_materials/blob/main/local_planner/local_planner_play_ground.gif" width="700" height="440"/>
</p>
