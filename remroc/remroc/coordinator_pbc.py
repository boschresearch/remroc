
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
import numpy as np
import json
from pathlib import Path

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ament_index_python.packages import get_package_share_directory

from nav2_msgs.action import ComputePathToPose, FollowPath

from scipy.spatial.transform import Rotation
from scipy.spatial import KDTree

import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient


'''
This node contains the code for the pbc coordinator.
'''

class ExperimentAsync(Node):

    def __init__(self):
        super().__init__('experiment_async')


        self.declare_parameter("map_name", 'none')
        self.map_name = self.get_parameter("map_name").value

        self.declare_parameter("number_of_humans", 0)
        self.number_of_humans = self.get_parameter('number_of_humans').value

        self.declare_parameter('sample', 0)
        self.sample = self.get_parameter('sample').value

        self.declare_parameter("experiment_yaml", "exp_0_depot.yaml")
        self.experiment_yaml = self.get_parameter("experiment_yaml").value

        # make the node use Simtime
        self.set_parameters([rclpy.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        self.pkg_remroc = get_package_share_directory('remroc')
        
        ##################################################################################################################################################################
        #IMPORTANT!!! This is a manual parameter which is dependent on the robot's radius. It represents the distance which two points in a path can have from each other before they are considdered to be a critical point.
        self.distance_threshold = 0.5
        ##################################################################################################################################################################

        # 2 variables to hold node times for metric computation
        self.node_start_time = self.get_clock().now()
        self.node_current_time = self.get_clock().now()
        
        # Load the information about the scenario. importantly the goal locations of the robots
        with open(Path(self.pkg_remroc).joinpath('params', self.experiment_yaml), 'r') as file:
            experiment_dict = yaml.safe_load(file)
        # locations for the MAPF problem.
        self.robots_goal_dict = {robot['type'] + '_' + str(robot['id']): robot['goal'] for robot in experiment_dict['robots']}

        # Robot Names:
        self.robot_names = []

        for robot in experiment_dict['robots']:
            robot_name = robot['type'] + '_' + str(robot['id'])
            self.robot_names.append(robot_name)
 
        #Setting up information dictionary, these will be filled by the code and then used to specifically distribute information to the respective robots
        self.robot_states = {robot_name: None for robot_name in self.robot_names}
        self.next_goals = {robot_name: None for robot_name in self.robot_names}

        self.robot_paths = {robot_name: None for robot_name in self.robot_names}
        self.robot_paths_todo = {robot_name: None for robot_name in self.robot_names}
        self.critical_sections = {robot_name: None for robot_name in self.robot_names}

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
            self.action_client_dict[robot_name] = ActionClient(self, FollowPath, f'{robot_name}/follow_path')

        self.compute_path_action_client_dict = {}
        for robot_name in self.robot_names:
            self.compute_path_action_client_dict[robot_name] = ActionClient(self, ComputePathToPose, f'{robot_name}/compute_path_to_pose')

        # Dictionary to hold the cmd_vel publishers
        # With these publishers one can directly access and overide the velocity comands of the individual robots.
        # i.e. if one wants to write logic that does not depend on the global or local planner of nav2, (emergency stop).
        self.cmd_vel_pub_dict = {}
        for robot_name in self.robot_names:
            self.cmd_vel_pub_dict[robot_name] = self.create_publisher(Twist, f'{robot_name}/cmd_vel', 10)




    # ################################ These three functions are part of the logic that calls the action server for computing a path, and then saving that path object in the dedicated dictionary. ######################
    def compute_robots_paths(self):

        for robot_name, action_client in self.compute_path_action_client_dict.items():
            self.get_logger().info(f'{robot_name}')

            goal_msg = ComputePathToPose.Goal()
            next_goal_pose = self.next_goals[robot_name]

            goal_msg.goal.header.frame_id = 'map'
            goal_msg.goal.pose.position.x = next_goal_pose[0]
            goal_msg.goal.pose.position.y = next_goal_pose[1]

            action_client.wait_for_server()
            robot_future = action_client.send_goal_async(goal_msg)
            robot_future.add_done_callback(lambda msg, in_robot_name=robot_name: self.compute_robots_paths_response_callback(msg, in_robot_name))

    def compute_robots_paths_response_callback(self, future, robot_name):
        goal_handle = future.result()
        self._get_results_future = goal_handle.get_result_async()
        self._get_results_future.add_done_callback(lambda msg, in_robot_name=robot_name: self.compute_robots_paths_result_callback(msg, in_robot_name))

    def compute_robots_paths_result_callback(self, future, robot_name):
        self.robot_paths[robot_name] = future.result().result.path
        self.robot_paths_todo[robot_name] = future.result().result.path
    # ######################################################################################################################################################################################################################

    def compute_critical_sections(self):
        '''
        This function computes the critical sections between paths and writes them into the self.critical_sections dictionary.
        '''

        kd_trees = {robot_name: None for robot_name in self.robot_names}
        for robot_name in self.robot_names:
            robot_path = [(point.pose.position.x, point.pose.position.y) for point in self.robot_paths_todo[robot_name].poses]
            if robot_path == []:
                robot_path.append((self.robot_states[robot_name].pose.pose.position.x, self.robot_states[robot_name].pose.pose.position.y))
            kd_trees[robot_name] = KDTree(robot_path)
        
        for robot_name in self.robot_names:
            other_robot_names = [x for x in self.robot_names if x != robot_name]
            self.critical_sections[robot_name] = {other_robot_name: [] for other_robot_name in other_robot_names}
            robot_path = [(point.pose.position.x, point.pose.position.y) for point in self.robot_paths_todo[robot_name].poses]
            if robot_path == []:
                robot_path.append((self.robot_states[robot_name].pose.pose.position.x, self.robot_states[robot_name].pose.pose.position.y))

            for other_robot in other_robot_names:
                close_points = kd_trees[other_robot].query_ball_point(robot_path, self.distance_threshold)
                # Close points is a list as long as the path. each entry is a list that contains a list. This list contains all indexis of points from other_path which are closer then the threshold.
                critical_section = {'indices': [], 'close_points_range': []}
                for index, point in enumerate(close_points): #for each entry in close_points
                    if point != []: #if the point is close to one or more other points append that point to the critical section, also note down the close points for later logic
                        critical_section['indices'].append(index)
                        critical_section['close_points_range'].append([point[0], point[-1]]) #append the first and the last point index, as it is allways in order anyways
                        if index != len(close_points)-1: #if this point is not the last point in the path
                            if close_points[index+1] == []: #check if the next point has any neibours. If not, the critical section is over, gets appended, and a new empty structure is created.
                                self.critical_sections[robot_name][other_robot].append(critical_section)
                                critical_section = {'indices': [], 'close_points_range': []}
                        else: #if it is the last point of the path, the critical section is also over. 
                            self.critical_sections[robot_name][other_robot].append(critical_section)
                            critical_section = {'indices': [], 'close_points_range': []}

        # self.get_logger().info(f'{self.critical_sections}')        

    def compute_critical_section_type(self, critical_section):
        '''
        This function takes as input a critical section between two robots, and determines if it is a Type 0, or Type 1 section. 
        Type 0: the robots traverse the section in the same direction
        Type 1: the robots traverse the secion in opposite directions
        '''
        point_1 = critical_section['close_points_range'][0]
        point_2 = critical_section['close_points_range'][-1]
        if sum(point_1)/2 < sum(point_2)/2:
            return 0
        else:
            return 1
        
    def state_subscriber_callback(self, msg, robot_name):
        self.robot_states[robot_name] = msg
    
    def odom_subscriber_callback(self, msg, robot_name):
        time_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9

        linear_velocity_msg = msg.twist.twist.linear
        linear_velocity = (linear_velocity_msg.x, linear_velocity_msg.y)

        angular_velocity_msg = msg.twist.twist.angular
        angular_velocity = (angular_velocity_msg.x, angular_velocity_msg.y, angular_velocity_msg.z)
        self.result_dict['odometry'][robot_name].append({'time_stamp': time_stamp, 'linear_vel': linear_velocity, 'angular_vel': angular_velocity})

    def laser_subscriber_callback(self, msg, robot_name):
        time_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9

        min_range = min(msg.ranges)
        self.result_dict['min_range'][robot_name].append({'time_stamp': time_stamp, 'min_range': min_range})
    
    def update_next_goal(self):
        for robot_name, robot_goal in self.robots_goal_dict.items():
            self.next_goals[robot_name] = robot_goal

    def compute_paths_todo(self):
        '''
        This function computes the remainding part of the path the robot still has to follow.
        it creates a copy of the original global path, and then finds the point on that path which the robot is closest to. 
        The path is then truncated accoardingly.
        '''

        self.robot_paths_todo = self.robot_paths.copy()
        for robot in self.robot_paths_todo.keys():
            index = 0
            dist = 10000000
            robot_state_msg = self.robot_states[robot]
            robot_state = np.array([robot_state_msg.pose.pose.position.x, robot_state_msg.pose.pose.position.y])
            robot_path = [(point.pose.position.x, point.pose.position.y) for point in self.robot_paths_todo[robot].poses]
            for i in range(len(robot_path)):
                dist_i = np.linalg.norm(robot_state-robot_path[i])
                if dist_i < dist:
                    dist = dist_i
                    index = i
            self.robot_paths_todo[robot].poses = self.robot_paths_todo[robot].poses[index:]

        return

    def send_action_goal(self):

        # Loop through the action_client_dict and set the action server goals for each robot
        for robot_name, action_client in self.action_client_dict.items():

            # create the goal-object and get the remaining path
            goal_msg = FollowPath.Goal()
            robot_path = self.robot_paths_todo[robot_name]

            index = len(robot_path.poses)

            # get the critical sections and loop through them
            robot_critical_sections = self.critical_sections[robot_name]
            for other_robot, robot_robot_critical_section in robot_critical_sections.items(): # loop through all robots and all of their critical sections
                for section in robot_robot_critical_section: # loop the critical sections of an individual robot
                    first_conflict_index = section['indices'][0]
                    first_conflict_close_point_range = section['close_points_range'][0]
                    last_conflict_close_point_range = section['close_points_range'][-1]
                    critical_section_type = self.compute_critical_section_type(section)

                    # Evaluate section type then find the closest point and the first index of conflict depended on the type of the section.
                    if critical_section_type == 0:
                        # This evaluates who is closer to the critical section and establishes presidence
                        min_conflict_close_point = min([x[0] for x in section['close_points_range']])
                        if first_conflict_index >= min_conflict_close_point:
                            if min_conflict_close_point < index:
                                index = max(0,first_conflict_index-10) # the -10 is there to have some safetydistance
                    
                    if critical_section_type == 1:
                        min_conflict_close_point = min([x[0] for x in section['close_points_range']])
                        if first_conflict_index >= min_conflict_close_point:
                            if min_conflict_close_point < index:
                                index = max(0,first_conflict_index-10)

            # truncate the path_todo up untill the first point of the relevant critical section.
            goal_msg.path.poses = robot_path.poses[:index]
            goal_msg.path.header.frame_id = 'map'

            goal_msg.goal_checker_id = "general_goal_checker"
            goal_msg.controller_id = "FollowPath"

            # send the final path to the robot
            action_client.wait_for_server()
            action_client.send_goal_async(goal_msg)

    def check_robots_at_goal(self) -> bool:
        '''
        Checks wether the robots are at the goal state and if so collect metrics. 
        '''
        self.node_current_time = self.get_clock().now()
        robots_at_goal = True
        for robot_name, robot_state in self.robot_states.items():
            # these are the robot states and goals in meters
            robot_state_array = np.array([robot_state.pose.pose.position.x, robot_state.pose.pose.position.y])
            robot_goal_array = np.array(self.next_goals[robot_name])
            # Robot is considered at the goal state, if the distance is smaller then 0.5 meters
            robot_at_goal = np.linalg.norm(robot_state_array-robot_goal_array) < 0.5

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
    experiment.node_start_time = experiment.get_clock().now()

    robots_at_goal = False

    # get all robot states initially
    all_states_recieved = False
    while not all_states_recieved:
        rclpy.spin_once(experiment)
        all_states_recieved=True
        for robot_state in experiment.robot_states.values():
            if robot_state is None: all_states_recieved = False

    experiment.get_logger().info(f'Response recieved.')

    # We update "next" goals. In this case it just sets the robots next goals to their final ones. 
    experiment.update_next_goal()

    # Using the global planner to get paths for each robot.
    experiment.compute_robots_paths()
    all_paths_done = False
    # Spin untill we have all paths
    while not all_paths_done:
        rclpy.spin_once(experiment)
        all_paths_done = True
        for path in experiment.robot_paths.values():
            if path == None:
                all_paths_done = False

    while not robots_at_goal:

        # This is the while loop that spins the node until the whole robot_states dictionary is filled for the first time, so the main while loop can start running.
        all_states_recieved = False
        while not all_states_recieved:
            rclpy.spin_once(experiment)
            all_states_recieved=True
            for robot_state in experiment.robot_states.values():
                if robot_state is None: all_states_recieved = False

        experiment.get_logger().info(f'Response recieved.')

        # Compute the segments between the current state of the robots and the goal
        experiment.compute_paths_todo()
        # Compute the segments only untill the next critical point
        experiment.compute_critical_sections()
        # Send the path segment to the robot
        experiment.send_action_goal()
        experiment.get_logger().info(f'Goal send.')

        # Checking if all robots are at their goal state, to potentialy break the loop and end the experiment
        robots_at_goal = experiment.check_robots_at_goal()
        # robots_at_goal = True
        experiment.node_current_time = experiment.get_clock().now()
        time_msg = (experiment.node_current_time - experiment.node_start_time).to_msg()
        time_sec = time_msg.sec + time_msg.nanosec*1e-9
        if time_sec >= 600:
            robots_at_goal = True

        # Clearing the robot_states-dictionary, so we make sure we recieve new states from all the robots
        experiment.robot_states = {robot_name: None for robot_name in experiment.robot_names}
        # experiment.robot_paths_todo = {robot_name: None for robot_name in experiment.robot_names}

    # Writing the results of the experiments to a json file at the specified location
    target_path = Path(f"results/pbc/{experiment.map_name}/{experiment.number_of_humans}/{experiment.sample}.json")
    target_path.parent.mkdir(exist_ok=True, parents=True)
    with target_path.open('w') as result_file:
        json.dump(experiment.result_dict, result_file, indent=4)

    experiment.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()