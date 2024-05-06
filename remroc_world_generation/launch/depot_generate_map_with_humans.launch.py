
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
import json

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from pathlib import Path
import numpy as np
from PIL import Image
from numpy import asarray

from launch import LaunchDescription
from launch.actions import EmitEvent, LogInfo, RegisterEventHandler, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import PathJoinSubstitution

# This gets the occupancy image which is used to generate the human trajectories
remroc_world_generation_pkg_path = get_package_share_directory('remroc_world_generation')
image_path = Path(remroc_world_generation_pkg_path).joinpath('worlds', 'maps', 'depot_humans_check.pgm')
image = Image.open(image_path).convert('L')
im = asarray(image)
im = np.flip(im, axis=0)

map_resolution = 0.04 # map resolution from the yamlfile. pixel*map_resolution = meter 
map_origin = [-15.1, -7.74]

def random_valid_state():
    '''
    This function randomly outputs a valid (not occupied) point dependend on the occupancy image
    '''

    valid = False
    while valid is False:
        x = np.random.randint(0, 754)
        y = np.random.randint(0, 380)
        if im[y, x] > 253:
            valid = True
    
    x = map_resolution*x
    y = map_resolution*y
    return x, y, np.array([x + map_origin[0], y + map_origin[1]])

def generate_start_end():
    '''
    This generates a set of valid start and goal positions.
    '''
    # start and goals for simple
    x_s, y_s, start = random_valid_state()
    x_e, y_e, end = random_valid_state()
    while np.linalg.norm(end-start) < 15:
        x_s, y_s, start = random_valid_state()
        x_e, y_e, end = random_valid_state()

    angle = np.arctan2(y_e - y_s,x_e - x_s)

    start = np.append(start, angle)
    end = np.append(end, angle)
    return start, end

def generate_launch_description():

    ld = LaunchDescription()

    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    nav2_bringup_launch = PathJoinSubstitution([nav2_bringup_dir, "launch", "bringup_launch.py"])
    human_trajectories_json_path = Path(remroc_world_generation_pkg_path).joinpath('worlds', 'human_trajectories.json')

    # This node listens to the planners status and terminates if it is ready.
    planner_server_ready_listener = Node(
        name="planner_ready_listener",
        package='remroc_world_generation',
        executable="planner_ready_listener",
    )

    map_odom_tf = Node(
        name="map_odom_tf",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
    )

    odom_baselink_tf = Node(
        name="odom_baselink_tf",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"]
    )

    # This launches a nav2 stack to generate global paths for the humans.
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_launch]),
        launch_arguments=[
        ("map", Path(remroc_world_generation_pkg_path).joinpath('worlds', 'maps', 'depot_humans.yaml').as_posix()),
        ("use_sim_time", 'false'),
        ("params_file", Path(remroc_world_generation_pkg_path).joinpath('params', 'human_nav2_params.yaml').as_posix())
        ]
    )

    ld.add_action(planner_server_ready_listener)
    ld.add_action(map_odom_tf)
    ld.add_action(odom_baselink_tf)
    ld.add_action(nav2_launch)

    # define the number of humans, and set the planner_server_ready_listener as first "check" node. 
    # the following loop creates a sequence of nodes, the start of each of which is conditioned on the termination of the previous one. 
    # this way we generate n trajectories, by calling the global planner n times sequentially.

    num_human = 20
    compute_path_to_pose_action_client_old= planner_server_ready_listener

    human_trajectories_json_path.unlink(missing_ok=True)
    with human_trajectories_json_path.open('a') as file:
        json.dump({}, file)

    for x in range(num_human):
        start, end = generate_start_end()

        # The node which queries the global path planner for a given start and goal location. 
        compute_path_to_pose_action_client = Node(
            name="compute_path_to_pose_action_client",
            package='remroc_world_generation',
            executable="compute_path_to_pose_client",
            parameters=[
            {"start": [start[0], start[1], start[2]]},
            {"goal": [end[0], end[1], end[2]]},
            {"human_id": x}
            ]
        )

        # This event handler launches the previous node if the one before that is finished
        my_event_handler = RegisterEventHandler(
            OnProcessExit(
                target_action=compute_path_to_pose_action_client_old,
                on_exit=[
                    LogInfo(msg="Planner server is ready. Sending Path compute request."),
                    compute_path_to_pose_action_client
                ]
            )
        )
        ld.add_action(my_event_handler)

        compute_path_to_pose_action_client_old = compute_path_to_pose_action_client

    # The world generation node contains code which puts the previously generated trajectories into a placeholder sdf. 
    # IMPORTANT: The final sdf will be writen to the worlds/sdfs directory in the -->>install<<-- folder of the package!!!
    world_generation_node = Node(
        name='world_generation_node',
        package='remroc_world_generation',
        executable='world_generation_node',
        parameters=[
            {'number_of_humans': num_human},
            {'map': 'depot'}
        ]
    )

    my_generation_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=compute_path_to_pose_action_client_old,
            on_exit=[
                LogInfo(msg='Paths Computed. Generating World.'),
                world_generation_node
            ]
        )
    )

    ld.add_action(my_generation_handler)

    my_shutdown_handler = RegisterEventHandler(
        OnProcessExit(
        target_action = world_generation_node,
        on_exit=[
            LogInfo(msg="World created. Terminating Nav2."),
            EmitEvent(event=Shutdown(reason="world created"))
        ]
        )
    )

    ld.add_action(my_shutdown_handler)

    return ld