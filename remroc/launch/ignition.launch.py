# Copyright (c) 2024 - for information on the respective copyright owner
# see the NOTICE file or the repository https://github.com/boschresearch/remroc/.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.



import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument, RegisterEventHandler, LogInfo, GroupAction, TimerAction, EmitEvent
from launch.event_handlers import OnExecutionComplete, OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, PythonExpression

from launch_ros.actions import Node


coordinator = 'coordinator_mapf' # The coordinator to use for the experiment. Must match the executable name of the ros2 node.
experiment_yaml = 'exp_0_simple.yaml' # The experiment yaml. This specifies the start and goal positions, type and id of the robots. Localted in the "params" folder.
world_name_default = 'simple' # The world name which is used. This should match the name of the maps and sdfs. as well as the world name inside the sdf. (Check simple and depot worlds for an example).
sample_default = '0' # The sample number which is used. The samples differe in the respective trajectories of the humans. Needs to match an existing world sdf.
number_of_humans_default = '5' # The number of humans which populate the environment. Needs to match an existing world sdf.

use_depot_mod = False # Wether or not to use the modified version of the depot environment, in which the "busy" area is blocked for the robots.


ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false']),
    DeclareLaunchArgument('world', default_value=world_name_default,
                          description='Ignition world type.'),
    DeclareLaunchArgument('number_of_humans', default_value=number_of_humans_default,
                          description='Number of humans in the world.'),
    DeclareLaunchArgument('sample', default_value=sample_default,
                          description='World Sample.'),
    
]

def generate_launch_description():

    # create launch configuration variables
    world_name = LaunchConfiguration('world')
    number_of_humans = LaunchConfiguration('number_of_humans')
    sample = LaunchConfiguration('sample')
    coordinator_sub = TextSubstitution(text=coordinator)
    if use_depot_mod:
        map_name = TextSubstitution(text='depot_mod')
    else:
        map_name = world_name

    # get package share folder paths
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg_remroc = get_package_share_directory('remroc')
    pkg_remroc_robots = get_package_share_directory('remroc_robots')

    
    # Set ignition resource path including the directory for the robot sdfs.
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            Path(pkg_remroc).joinpath('worlds').as_posix() + ':' + 
            Path(pkg_remroc_robots).joinpath('robots').as_posix()
            ])

    # Paths
    ign_gazebo_launch = PathJoinSubstitution(
        [pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'])

    # Ignition gazebo
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ign_gazebo_launch]),
        launch_arguments=[
            ('gz_args', [
                'sdfs/', world_name, '_', number_of_humans, '_', sample, '.sdf', #The world to launch. Make sure there exists a corresponding sdf in "worlds/sdfs"
                ' -v 4', #Verbose mode
                ' -r', #start unpaused
                ' --headless-rendering', # This helps with performance
            ])
        ]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(ign_resource_path)
    ld.add_action(ignition_gazebo) # Launching gazebo

    # Bridging the Ignition clock to the ros2clock so that the timestamps on the sensors line up.
    ld.add_action(
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=[
                ['/world/', world_name, '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
            ],
            remappings=[
                (['/world/', world_name, '/clock'], '/clock'),
            ],
        )
    )


    # A service node which exposes an interface to solve a MAPF problem.
    # This is only launched if the the coordination algorithm is mapf based. Currently all mapf_solver_servers use CBS.
    ld.add_action(
        Node(
        package='remroc_mapf_solver',
        executable='mapf_solver_server',
        name='mapf_solver_server',
        condition=IfCondition(PythonExpression(["'", coordinator_sub, "' in ['coordinator_mapf', 'coordinator_mapf_baseline']"])), 
        output='screen',
        parameters=[{'mapf_file_path': [Path(pkg_remroc).joinpath('params').as_posix(), '/mapf_', map_name, '.yaml']}]
        )
    )

    # Add map server
    ld.add_action(
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'yaml_filename': [Path(pkg_remroc).joinpath('worlds', 'maps').as_posix(),'/', map_name, '.yaml']}
            ]
        )
    )

    # We need to launch a nav2 lifecycle manager for the mapserver, as it is not in a namespace.
    ld.add_action(
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_node',
            output='screen',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'node_names': ['map_server']},
                {'autostart': True}
            ]
        )
    )

    # Loading the experiment yaml which contains the robot specifications
    with open(Path(pkg_remroc).joinpath('params', experiment_yaml)) as file:
        experiment_dict = yaml.safe_load(file)

    # Creating a nested list with the start, type and ids of the robots
    starts = [[x['start'], x['type'], x['id']] for x in experiment_dict['robots']]

    # This loop launches all robot specific nodes for each of the robot specified in the starts list.
    for i in range(len(starts)):

        # set name and spawn location
        robot_pose = starts[i][0]
        robot_description = starts[i][1]
        robot_id = starts[i][2]
        robot_name = f'{robot_description}_{robot_id}' # All robot components including the transform tree are launched inside a namespace <robot_name>

        # remapping the tf topics inside the respective namespace
        remappings = [
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ]

        # spawning actions for the individual robots.
        robot_launch_node = Node(
            package='ros_gz_sim',
            executable='create',
            name=f'launch_node_{robot_name}',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=['-world', world_name, '-file', Path(pkg_remroc_robots).joinpath('robots', robot_description, 'model.sdf').as_posix(), '-name', robot_name, '-x', str(robot_pose[0]), '-y', str(robot_pose[1]), '-Y', str(robot_pose[2])]
        )
        ld.add_action(robot_launch_node)

        # Add sensor transforms
        # read the sensor position dictionary needed for static transforms of the respective robot type
        with open(Path(pkg_remroc_robots).joinpath('config', robot_description, 'sensor_positions.yaml')) as file:
            sensor_positions = yaml.safe_load(file)

        # for each sensor specified in the sensor_positions.yaml, launch a static transform from the sensor to the base_link
        for sensor_dict in sensor_positions.values():
            ld.add_action(
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    namespace=robot_name,
                    output='screen',
                    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                    arguments=[
                        '--x', sensor_dict['x'],
                        '--y', sensor_dict['y'],
                        '--z', sensor_dict['z'],
                        '--yaw', sensor_dict['yaw'],
                        '--pitch', sensor_dict['pitch'],
                        '--roll', sensor_dict['roll'],
                        '--frame-id', 'base_link',
                        '--child-frame-id', f'{robot_name}/{sensor_dict["sensor_frame_id"]}',
                        ],
                    remappings=remappings
                )
            )


        # A ros-gazebo bridge is needed for the sensor messages as well as odometry and cmd_vel topics. At the time this is still manual.
        # TODO: Maybe automatize this, by adding another config yaml for each robot with a list of remappings. 
        ld.add_action(
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                namespace=robot_name,
                name='ros_gz_bridge',
                output='screen',
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                arguments=[
                    ['/model/', robot_name, '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist'],
                    ['/model/', robot_name, '/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry'],
                    ['/world/', world_name, '/model/', robot_name, '/link/imu_link/sensor/imu/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU'],
                    ['/world/', world_name, '/model/', robot_name, '/link/scan_omni/sensor/scan_omni/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],
                    ['/world/', world_name, '/model/', robot_name, '/link/scan_omni/sensor/scan_omni/scan/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'],
                ],
                remappings=[
                    (['/model/', robot_name, '/cmd_vel'], ['/', robot_name, '/cmd_vel']),
                    (['/model/', robot_name, '/odometry'], ['/', robot_name, '/odometry']),
                    (['/world/', world_name, '/model/', robot_name, '/link/imu_link/sensor/imu/imu'], ['/', robot_name, '/imu']),
                    (['/world/', world_name, '/model/', robot_name, '/link/scan_omni/sensor/scan_omni/scan'], ['/', robot_name, '/laser_scan']),
                    (['/world/', world_name, '/model/', robot_name, '/link/scan_omni/sensor/scan_omni/scan/points'], ['/', robot_name, '/point_cloud']),
                ],
            )
        )

        
        # This group action launches the local navigation components of each robot
        group_action = GroupAction(
            actions = [
              Node( # robot state-estimation using an extended kalman filter.
                package='robot_localization',
                executable='ekf_node',
                namespace=robot_name,
                respawn=True,
                respawn_delay=2.0,
                name='ekf_node',
                output='screen',
                parameters=[
                    Path(pkg_remroc_robots).joinpath('config', robot_description, 'ekf.yaml').as_posix(),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}
                ],
                remappings=remappings
            ),
            Node( # robot localization using AMCL
                package='nav2_amcl',
                executable='amcl',
                namespace=robot_name,
                respawn=True,
                respawn_delay=2.0,
                name='amcl_node',
                output='screen',
                parameters=[
                    Path(pkg_remroc_robots).joinpath('config', robot_description, 'amcl.yaml').as_posix(),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'x': str(robot_pose[0])},
                    {'y': str(robot_pose[1])},
                    {'yaw': str(robot_pose[2])},
                    {'set_initial_pose': True},
                    {'initial_pose': {'x': robot_pose[0], 'y': robot_pose[1], 'yaw': robot_pose[2]}}
                ],
                remappings=remappings
            ),
            Node( # robot controller, currently MPPI
                package='nav2_controller',
                executable='controller_server',
                namespace=robot_name,
                name='controller_server_node',
                respawn=True,
                respawn_delay=2.0,
                output='screen',
                parameters=[
                    Path(pkg_remroc_robots).joinpath('config', robot_description, 'controller_server.yaml').as_posix(),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                ],
                remappings=remappings + 
                    [
                        # ('cmd_vel', 'cmd_vel_nav'), 
                        ('/laser_scan', f'/{robot_name}/laser_scan'),
                        ('/point_cloud', f'/{robot_name}/point_cloud'),
                        ('/trajectories', f'/{robot_name}/trajectories'),
                    ]
            ),
            Node( # robot global planner, currently Hybrid A*
                package='nav2_planner',
                executable='planner_server',
                namespace=robot_name,
                name='planner_server_node',
                respawn=True,
                respawn_delay=2.0,
                output='screen',
                parameters=[
                    Path(pkg_remroc_robots).joinpath('config', robot_description, 'planner_server.yaml').as_posix(),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                ],
                remappings=remappings + 
                    [
                        ('/laser_scan', f'/{robot_name}/laser_scan'),
                        ('/point_cloud', f'/{robot_name}/point_cloud'),
                    ]
            ),
            Node( # behaviours
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server_node',
                namespace=robot_name,
                respawn=True,
                respawn_delay=2.0,
                output='screen',
                parameters=[
                    Path(pkg_remroc_robots).joinpath('config', robot_description, 'behavior_server.yaml').as_posix(),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                ],
                remappings=remappings
            ),
            Node( # behaviour tree
                package='nav2_bt_navigator',
                executable='bt_navigator',
                namespace=robot_name,
                name='bt_navigator_node',
                respawn=True,
                respawn_delay=2.0,
                output='screen',
                parameters=[
                    Path(pkg_remroc_robots).joinpath('config', robot_description, 'bt_navigator.yaml').as_posix(),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}
                ],
                remappings=remappings
            ),
            Node( # Lifecycle manager to launch all the robot specific nodes. Make sure to add/remove any chances under "node_names"
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                namespace=robot_name,
                name='lifecycle_manager_node',
                output='screen',
                parameters=[
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'node_names': ['amcl_node', 'controller_server_node', 'planner_server_node', 'behavior_server_node', 'bt_navigator_node']},
                    {'autostart': True}
                ],
            ),
            ]
        )

        # Add robot_state publisher. This takes the /odometry/filtered topic and republishes it in the map frame on /robot_state
        # This is required as one can not call a transform and specify a namespace for the TF tree. 
        robot_state_publisher_node = TimerAction(actions=[Node(
            package='remroc',
            executable='robot_state_publisher',
            namespace=robot_name,
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=remappings,
        )], period=5.)

        # Launch the nav2 nodes only after the robot has been created
        launch_nav2_nodes = RegisterEventHandler(
            OnProcessExit(
            target_action=robot_launch_node,
            on_exit=TimerAction(actions=[group_action, robot_state_publisher_node], period=1.)
            )
        )
        ld.add_action(launch_nav2_nodes)
        
        # The loop that launches the individual robots and their specific components ends here.


    robot_experiment_node = Node( 
        package='remroc',
        executable=coordinator,
        name='coordinator_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'map_name': map_name, 
                     'number_of_humans': number_of_humans,
                     'sample': sample,
                     'experiment_yaml': experiment_yaml}],
    )
    # This launches the respective coordinator node when all robots are launched. as it takes a while for all nav2 components to fully start up there is a delay.
    # The delay should depends on the number of robots and the hardware on which the experiment are run. 
    launch_experiment_node = RegisterEventHandler(
        OnExecutionComplete(
        target_action=group_action,
        on_completion=TimerAction(actions=[robot_experiment_node], period=30.)
        )
    )
    ld.add_action(launch_experiment_node)

    # We did not use this, as we terminated the experiments in the shell manually. 
    my_shutdown_handler = RegisterEventHandler(
        OnProcessExit(
        target_action=robot_experiment_node,
        on_exit=[
            LogInfo(msg="Experiment done. Terminating simulation."),
            EmitEvent(event=Shutdown(reason="Experiment Done"))
        ]
        )
    )
    # ld.add_action(my_shutdown_handler)

    return ld


