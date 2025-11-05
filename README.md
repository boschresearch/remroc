# Realistic Multi Robot Coordination

We introduce Realistic Multi Robot Coordination (REMROC). REMROC is a simulation and benchmark framework for multi-robot coordination methods and utilizes a physics based simulation and state-of-the-art robotic software tools. 

It is based on ROS 2 Humble and Gazebo Fortress and uses Navigation 2 for the navigation stack of individual robots. 

We recommend general knowledge of ROS 2 and the components of Navigation 2 when working with this framework.

## Installation

REMROC consists of multiple ROS 2 packages and can be installed simply by cloning the repository and building the packages in a ROS 2 workspace. 
To ensure that all preconditions are met please check that your system fulfils the following prerequisites:

1. installation of ROS 2 Humble - [Instructions](https://docs.ros.org/en/humble/Installation.html)
2. installation of Gazebo Ignition (version Fortress) - [Instructions](https://gazebosim.org/docs/fortress/install)
3. install colcon 
```
sudo apt install python3-colcon-common-extensions
```
4. Either use rosdep to install the required ROS 2 packages or the following packages manually: 
```
sudo apt install ros-humble-ros-ign-gazebo ros-humble-ros-gz-bridge ros-humble-navigation2 ros-humble-robot-localization
```
5. Move the contents of this repository into your ROS 2 workspace, remember to source the general ROS 2 installation ```source \opt\ros\humble\setup.bash```, and build the packages with ```colcon build``` .


## Start an experiment

To start an experiment, make sure the parameters in the launchfile [ignition.launch.py](./remroc/launch/ignition.launch.py) are defined properly. 

Remember to also source the respective workspace you have built the packages in, then you can start the specified experiment with 
```
ros2 launch remroc ignition.launch.py
``` 

## Contents

The REMROC framework consists of the following ROS 2 packages, some of which are interdependent: 
- The [remroc](./remroc/) package contains the central launchfile, the yaml files which describe the scenario setup, the simulation environments to be used, and the multi-robot coordination nodes.
- The [remroc_robots](./remroc_robots/) package contains the definitions of robots which can be used in the framework, including the config files of their respective navigation stacks.
- The [remroc_world_generation](./remroc_world_generation) package is used to generate SDF files of simulation environments populated with humans. It can be modified to introduce an arbitrary number of humans into any Gazebo world, specified as an sdf file.
- The [remroc_cbs_library](./remroc_cbs_library/) C++-package wraps functions from the [libMultiRobotPlanning](https://github.com/whoenig/libMultiRobotPlanning) repository to be used in a ROS 2 node.
- The [remroc_mapf_solver](./remroc_mapf_solver/) C++-package contains a service server node, which can be queried to solve a given instance of a mapf problem.
- The [remroc_interfaces](./remroc_interfaces/) package contains some custom service and topic message types which are used in the other packages. 

## Customization
REMROC allows for a high degree of customizability in mainly three parts:

- The multi-robot coordination method or algorithm
- The simulated environment and humans
- The robots

In the following we will explain the main launchfile and these individual components, how they integrated and how one can modify them individually.

### Launch file

The framework is started by launching the launchfile [ignition.launch.py](./remroc/launch/ignition.launch.py) from the remroc package. At the beginning of the launchfile one can specify the following things: 

- The multi-robot coordination algorithm to be used.
- The experiment setup. This is done through the experiment yaml file (e.g. [remroc/params/exp_0_simple.yaml](./remroc/params/exp_0_simple.yaml))specifies the number of robots, their type, start and goal positions.
- The world/environment which is going to be used.
- The number of humans in the world/environment. 
- The sample number. (in case one wants multiple instances of the same world with the same number of humans, but with the humans following different trajectories)

#### Remember the following dependencies when modifying the launchfile:
- The start and goal locations of the robots specified in the experiment yaml file are given in the environment coordinate system.
- The coordinator needs to correspond to a properly defined coordinator node in the remroc package. 
- The robot type needs to be properly defined in the remroc_robots package, 
- For the world name, the number of humans, and the sample number, there needs to be a corresponding SDF world defined in [remroc/worlds/sdfs/](./remroc/worlds/sdfs/), that fits these parameters. 

### Multi-robot coordination method

To introduce a new multi-robot coordination method or algorithm, one needs to define it in a ROS 2 node in [remroc/remroc/](./remroc/remroc). We provide a [template](./remroc/remroc/coordinator_template.py) as a starting point for anyone who wishes to implement a new coordination method. 

The template implements subscribers to robot odometry and some sensors, like the laser scanner, to obtain robot information. 
It also implements interfaces to control the robots. This can be done either by directly publishing control commands or utilizing action servers which are provided by the robot's navigation stack.

#### Remember the following dependencies when implementing a new coordination method:

- Robot information interface: The topics to which the coordination method subscribes for the robot information need to be defined with the correct name and message type. Also, if you want to obtain information directly from a gazebo sensor (e.g. a camera or laser scanner) there needs to be a ros2_gazebo bridge defined in the [launchfile](./remroc/launch/ignition.launch.py) to make the Gazebo Ignition messages available to ROS 2.
- Robot control interface: As the robots recieve their control commands only from the coordination node, it is important that the node refers to the appropriate topic/service/action. While one can use the highlevel action servers provided by navigation 2 behaviour tree, one can also directly trigger global or local planners and even directly publish control commands to the robots. See [coordinator_mapf.py](./remroc/remroc/coordinator_mapf.py) or [coordinator_pbc.py](./remroc/remroc/coordinator_pbc.py) for different examples of how this can be realized.
- External computational models: There may be external computational models which one wants to utilize for coordination. Examples may be an efficient mapf solver as [coordinator_mapf.py](./remroc/remroc/coordinator_mapf.py), or a learning based policy which returns coordination actions based on some information of the robot fleet. 
If such external computational models are used, one might need to include additional ROS 2 service or action servers, similar to [remroc_mapf_solver/src/mapf_solver_server.cpp](./remroc_mapf_solver/src/mapf_solver_server.cpp). Make sure their launch is included in the [launchfile](./remroc/launch/ignition.launch.py) and the topics and message types are alligned. 

### Environment and humans

The environments in REMROC are currently fixed sdf files. We utilize the agent class with a human skeleton and mesh to include humans which move along predetermined paths. The final sdf files need to be placed in [remroc/world/sdfs/](./remroc/worlds/sdfs/). The naming convention we follow is {_mapname_}\_{_number_of_humans_}\_{_sample_numbers_}.sdf.
In addition to the sdf file itself, the nav2 map server requires a .pgm occupancy map and a corresponding yaml file, which need to be placed in the [src/remroc/world/maps/](./remroc/worlds/maps/) directory and named with {_mapname_}.pgm and {_mapname_}.yaml.

The tutorial on actor models in gazebo ignition can be found [here](https://gazebosim.org/docs/fortress/actors)

The [remroc_world_generation](./remroc_world_generation/) package provides a starting point to populate an existing map with humans that move on preexisting trajectories. In the respective launchfiles we utilize a navigation 2 global path planner to generate paths between random start and goal positions. Currently there is no way to do this out-of-the-box for arbitrary maps and number of humans, so you will need to add specific launchfiles and auxilery code if you want to generate trajectories for different environments.

### Robots

Using a robot type in REMROC, requires a Gazebo model and a local navigation stack of the respective robot. We use a [Navigation2](https://navigation.ros.org/index.html) stack for the robots. To add a new type of robot, create a folder with the robot name in [remroc_robots/robots/](./remroc_robots/robots/) containing the model.sdf and required meshfiles. Create another folder with the robot name in  [remroc_robots/config](./remroc_robots/config/), containing the config files for the navigation stack. 

#### Remember the following dependencies when adding new robot types:

- Required sensors: The navigation stack and coordination methods may require certain sensor information to be available on certain topics (e.g. AMCL looks for a laser scan on a topic defined in the config file [remroc_robots/config/SmallDeliveryRobot/amcl.yaml](./remroc_robots/config/SmallDeliveryRobot/amcl.yaml)). Dependent on what kind of navigation stack you want to run on your robot you need to make sure that the respective sensors are defined in the model file and that there is a ros2_gazebo bridge launched in the [launchfile](./remroc/launch/ignition.launch.py) to bridge messages from Gazebo to ROS 2. 
- Navigation components: Make sure that all the required navigation components for the robots are launched in the main launchfile.


## Citation

If you find our code or paper useful, please cite

```bibtex
@inproceedings{heuer2024benchmarking,
  title={Benchmarking Multi-robot coordination in realistic, unstructured human-shared environments},
  author={Heuer, Lukas and Palmieri, Luigi and Mannucci, Anna and Koenig, Sven and Magnusson, Martin},
  booktitle={2024 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={14541--14547},
  year={2024},
  organization={IEEE}
}
```

## Known Issues

Simulating multiple robots and their navigation stacks requires a considerable amount of computational resources. 
To resolve performance issues we recommend reducing the number of robots in the experiment.

The setuptools library may need to be downgraded to build the packages properly. 

Navigation 2 may need to be build from source (humble branch) to include all the required packages, specifically the MPPI controller.

Gazebo can cause problems sometimes. in case the simulation does not start up as expected, try to verify that gazebo works properly. e.g. run ign gazebo shapes.sdf.

## License

REMROC is open-sourced under the Apache-2.0 license. See the
[LICENSE](LICENSE) file for details.

For a list of other open source components included in REMROC, see the
file [3rd-party-licenses.txt](3rd-party-licenses.txt).
