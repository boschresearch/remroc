
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



import sys 
import os
import yaml
import numpy as np
import json
import time
from pathlib import Path

from remroc_interfaces.srv import RemrocMapfsolver
from remroc_interfaces.msg import RemrocPose2darray
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from ament_index_python.packages import get_package_share_directory

from scipy.spatial.transform import Rotation

import functools

import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient


'''
This Node is a template of how one can set up a certain coordination algorithm experiment. 
It contains the basic logic that subscribes to the relevant topics and action servers.
Please use this code to get started with coding your own logic for sending intermidiate goals to the robots. 
Some knowledge and experience with ROS2 systems is recommended. 
'''

class ExperimentAsync(Node):

    def __init__(self):
        super().__init__('experiment_async')

        self.declare_parameter("world_name", 'none')
        self.world_name = self.get_parameter("world_name").value

        self.declare_parameter("number_of_humans", 0)
        self.number_of_humans = self.get_parameter('number_of_humans').value

        self.declare_parameter('sample', 0)
        self.sample = self.get_parameter('sample').value

        self.declare_parameter("experiment_yaml", "exp_0_depot.yaml")
        self.experiment_yaml = self.get_parameter("experiment_yaml").value

        # make the node use Simtime
        self.set_parameters([rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.pkg_remroc = get_package_share_directory('remroc')
        
        # 2 variables to hold node times for metric computation
        self.node_start_time = self.get_clock().now()
        self.node_current_time = self.get_clock().now()
        
        # Load the information about the scenario; robot types, start and goal states.
        with open(Path(self.pkg_remroc).joinpath('params', self.experiment_yaml), 'r') as file:
            experiment_dict = yaml.safe_load(file)

        # Dictionary with the goal states of the robots
        # Keys are the identifiers which are also used as ros2 namespaces for the robots
        # Values are the robots goal states
        self.robots_goal_dict = {robot['type'] + '_' + str(robot['id']): robot['goal'] for robot in experiment_dict['robots']}

        # Robot Names:
        self.robot_names = []

        for robot in experiment_dict['robots']:
            robot_name = robot['type'] + '_' + str(robot['id'])
            self.robot_names.append(robot_name)
 
        #Setting up information dictionary, these will be filled by the code and then used to specifically distribute information to the respective robots
        self.robot_states = {robot_name: None for robot_name in self.robot_names}
        self.next_goals = {robot_name: None for robot_name in self.robot_names}

        # create the dictionary that will hold all the metrics
        self.result_dict = {
            'time_to_goal': {robot_name: None for robot_name in self.robot_names},
            'min_range': {robot_name: [] for robot_name in self.robot_names},
            'odometry': {robot_name: [] for robot_name in self.robot_names},
        }

        # Create a subscriber to each of the robot_state topics
        # The state_subscriber_callback, takes in the respective robot name and then fills the response in the robot_states dictionary. 
        # IMPORTANT: When calling "node.spin_once" there is no guarrantee to recieve information from all robots!
        #            One needs to call node.spin_once in a while-loop untill the whole robot_states-dictionary is filled.
        for robot_name in self.robot_names:
            self.create_subscription(
                Odometry,
                f'{robot_name}/robot_state',
                lambda msg, in_robot_name=robot_name: self.state_subscriber_callback(msg, in_robot_name),
                10
            )

        # Subscribers to the odometry/filtered topic to collect some metrics
        for robot_name in self.robot_names:
            self.create_subscription(
                Odometry,
                f'{robot_name}/odometry/filtered',
                lambda msg, in_robot_name=robot_name: self.odom_subscriber_callback(msg, in_robot_name),
                10
            )

        # Subscribers to the odometry/filtered topic to collect some metrics
        for robot_name in self.robot_names:
            self.create_subscription(
                LaserScan,
                f'{robot_name}/laser_scan',
                lambda msg, in_robot_name=robot_name: self.laser_subscriber_callback(msg, in_robot_name),
                10
            )

        # Dictionary to hold the action clients. These interface the nav2 action servers that allow to navigate to a given pose.
        self.action_client_dict = {}
        for robot_name in self.robot_names:
            self.action_client_dict[robot_name] = ActionClient(self, NavigateToPose, f'{robot_name}/navigate_to_pose')

        # Dictionary to hold the cmd_vel publishers
        # With these publishers one can directly access and overide the velocity comands of the individual robots.
        # i.e. if one wants to write logic that does not depend on the global or local planner of nav2, (emergency stop).
        self.cmd_vel_pub_dict = {}
        for robot_name in self.robot_names:
            self.cmd_vel_pub_dict[robot_name] = self.create_publisher(Twist, f'{robot_name}/cmd_vel', 10)


    def state_subscriber_callback(self, msg, robot_name):
        self.robot_states[robot_name] = msg
    
    def odom_subscriber_callback(self, msg, robot_name):
        time_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9

        linear_velocity_msg = msg.twist.twist.linear
        linear_velocity = (linear_velocity_msg.x, linear_velocity_msg.y)

        angular_velocity_msg = msg.twist.twist.angular
        angular_velocity = (angular_velocity_msg.x, angular_velocity_msg.y, angular_velocity_msg.z)
        # Saves odometry to the result dictionary
        self.result_dict['odometry'][robot_name].append({'time_stamp': time_stamp, 'linear_vel': linear_velocity, 'angular_vel': angular_velocity})

    def laser_subscriber_callback(self, msg, robot_name):
        time_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9

        min_range = min(msg.ranges)
        # Saves the min-range of the laserscanner to the result dictionary
        self.result_dict['min_range'][robot_name].append({'time_stamp': time_stamp, 'min_range': min_range})
    
    def update_next_goal(self):
        # IMPORTANT: As this is a template, there will not be any logic to compute the intermediate goals for the individual robots
        # we will just set the next_goal equal to the global goal
        for robot_name, robot_goal in self.robots_goal_dict.items():
            self.next_goals[robot_name] = robot_goal


    def send_action_goal(self):

        # Loop through the action_client_dict and set the action server goals
        # We are just taking all the goals from the next_goals-dictionary and sending them to the action servers
        for robot_name, action_client in self.action_client_dict.items():

            goal_msg = NavigateToPose.Goal()
            next_goal_pose = self.next_goals[robot_name]
            
            goal_msg.pose.header.frame_id = 'map'

            goal_msg.pose.pose.position.x = next_goal_pose[0]
            goal_msg.pose.pose.position.y = next_goal_pose[1]

            action_client.wait_for_server()
            action_client.send_goal_async(goal_msg)

    def check_robots_at_goal(self) -> bool:
        '''
        Checks wether the robots are at the goal state and if so collect metrics. 
        '''
        self.node_current_time = self.get_clock().now()
        robots_at_goal = True

        # Compare the current and goal states for each robot.
        for robot_name, robot_state in self.robot_states.items():
            robot_state_array = np.array([robot_state.pose.pose.position.x, robot_state.pose.pose.position.y])
            robot_goal_array = np.array(self.next_goals[robot_name])
            # Robot is considered at the goal state, if the distance is smaller then 0.5 meters
            robot_at_goal = np.linalg.norm(robot_state_array-robot_goal_array) < 0.5
            # Save robots time-to-goal to the metrics dictionary
            if robot_at_goal:
                if (self.result_dict['time_to_goal'][robot_name] == None):
                    time_msg = (self.node_current_time - self.node_start_time).to_msg()
                    self.result_dict['time_to_goal'][robot_name] = time_msg.sec + time_msg.nanosec*1e-9
            else:
                robots_at_goal = False

        return robots_at_goal
            
def main():
    rclpy.init()

    experiment = ExperimentAsync()

    robots_at_goal = False
    while not robots_at_goal:
        # IMPORTANT: This while loop is admitedly a bit useless, as the action server already repeatedly calls the global planner of nav2.
        # it is still here, for illustration purposes. If one wants to implement coordination logic to sent different intermediate goals to the robots, this loop will be needed to run it iteratively.

        # This is the while loop that spins the node until the whole robot_states dictionary is filled for the first time, so the main while loop can start running.
        all_states_recieved = False
        while not all_states_recieved:
            rclpy.spin_once(experiment)
            all_states_recieved=True
            for robot_state in experiment.robot_states.values():
                if robot_state is None: all_states_recieved = False

        experiment.get_logger().info(f'Response recieved.')

        # We update the next goal, and then send it to the action server so the robots will navigate to the given goalstate
        experiment.update_next_goal()
        experiment.send_action_goal()
        experiment.get_logger().info(f'Goal send.')

        # Checking if all robots are at their final goal state, to potentialy break the loop and end the experiment
        robots_at_goal = experiment.check_robots_at_goal()


        time_msg = (experiment.node_current_time - experiment.node_start_time).to_msg()
        time_sec = time_msg.sec + time_msg.nanosec*1e-9
        if time_sec >= 300:
            robots_at_goal = True
        # Clearing the robot_states-dictionary, so we make sure we recieve new states from all the robots
        experiment.robot_states = {robot_name: None for robot_name in experiment.robot_names}

    # Writing the results of the experiments to a json file at the specified location
    target_path = f"results/template/{experiment.world_name}/{experiment.number_of_humans}/{experiment.sample}.json"
    with open(target_path, 'w+') as result_file:
        json.dump(experiment.result_dict, result_file, indent=4)

    experiment.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()