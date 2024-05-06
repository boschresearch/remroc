
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

from remroc_interfaces.srv import RemrocMapfsolver
from remroc_interfaces.msg import RemrocPose2darray
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateThroughPoses
from ament_index_python.packages import get_package_share_directory

from scipy.spatial.transform import Rotation

import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient


'''
This node contains the code for the iterative mapf coordinator.
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

        self.mapf_yaml = 'mapf_' + self.map_name + '.yaml'

        self.pkg_remroc = get_package_share_directory('remroc')
        
        # 2 variables to hold node times for metric computation
        self.node_start_time = self.get_clock().now()
        self.node_current_time = self.get_clock().now()

        # Define the minimum replanning interval for the mapf solver 
        replanning_time_sec = 0.55
        self.target_coordinator_loop_time = rclpy.duration.Duration(seconds=replanning_time_sec)
        
        # Load the information about the scenario
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

        self.robot_centroids = {robot['id']: None for robot in experiment_dict['robots']}
        self.latest_mapf_solution = {robot['id']: None for robot in experiment_dict['robots']}

        # create the dictionary that will hold all the metrics
        self.result_dict = {
            'time_to_goal': {robot['id']: None for robot in experiment_dict['robots']},
            'target_coordinator_loop_time': replanning_time_sec,
            'min_range': {robot['id']: [] for robot in experiment_dict['robots']},
            'coordinator_loop_time': [],
            'odometry': {robot['id']: [] for robot in experiment_dict['robots']},
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

        # create mapf_solver client and request message
        self.cli = self.create_client(RemrocMapfsolver, 'mapf_solver')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for mapf_solver_server service to become available.')
        self.mapf_solver_request = RemrocMapfsolver.Request()

        # Dictionary to hold the action clients
        self.action_client_dict = {}
        for robot_name in self.robot_names:
            self.action_client_dict[robot_name] = ActionClient(self, NavigateThroughPoses, f'{robot_name}/navigate_through_poses')

        # Dictionary to hold the cmd_vel publishers
        # With these publishers one can directly access and overide the velocity comands of the individual robots.
        # i.e. if one wants to write logic that does not depend on the global or local planner of nav2, (emergency stop).
        self.cmd_vel_pub_dict = {}
        for robot_name in self.robot_names:
            self.cmd_vel_pub_dict[robot_name] = self.create_publisher(Twist, f'{robot_name}/cmd_vel', 10)

        # This information is relevant for the gridmap abstraction used when generating the mapf_.....yaml. This needs to match the information used in the remroc_world_geenration package when creating the mapf_...yaml file.
        if self.map_name in ['depot', 'depot_mod']:
            ### depot 
            map_resolution = 0.04 # map resolution from the yamlfile. pixel*map_resolution = meter 
            grid_cell_size = 29 # The size of one grid cell in pixels
            x_offset = 4 # offset of the first grid cell in pixel in x direction
            y_offset = 14 # offset of the first grid cell in pixel in y direction. This is different from the x direction because the y axis in inverted. (in the grid generation code its 5 for x and y)
            map_origin = [-15.1, -7.74]

        elif self.map_name == 'simple':
            ### simple 
            map_resolution = 0.05 # map resolution from the yamlfile. pixel*map_resolution = meter 
            grid_cell_size = 29 # The size of one grid cell in pixels
            x_offset = 15 # offset of the first grid cell in pixel in x direction
            y_offset = 15 # offset of the first grid cell in pixel in y direction. This is different from the x direction because the y axis in inverted. (in the grid generation code its 5 for x and y)
            map_origin = [-5.08, -5.09]
        
        else: 
            raise AssertionError('Unknown mapname! Could not launch experiment node.')

        # loading the gridmap yaml with gridsize and obstacles
        with open(Path(self.pkg_remroc).joinpath('params', self.mapf_yaml), 'r') as mapf_file:
            yaml_file = yaml.safe_load(mapf_file)
        
        # Dictionaries for all the centroids in "centroid[index]: centroid[meters]"
        self.centroid_dict = {}

        # Filling the centroid dict
        temp_index = yaml_file['map']['dimensions'][1]-1
        for x in range(yaml_file['map']['dimensions'][0]):
            for y in range(yaml_file['map']['dimensions'][1]):
                #IMPORTANT I am inverting the second index because this is the way that the cbs algorithm expects it. This is subject to the convention of the mapf_solver_algorithm.
                self.centroid_dict[f'{x}, {-(y-temp_index)}'] = [map_resolution*(x*grid_cell_size + x_offset + grid_cell_size/2) + map_origin[0], map_resolution*(y*grid_cell_size + y_offset + grid_cell_size/2) + map_origin[1]]

        # Removing all centroids that are acctually obstacles
        for obstacle in yaml_file['map']['obstacles']:
            del self.centroid_dict[str(obstacle)[1:-1]]

        self.robots_goal_centroids = {}

        temp_centroid_dict = self.centroid_dict.copy()

        for robot, goal in self.robots_goal_dict.items():
            # starting values that are so unrealistic that they will be overwritten every time
            robot_centroid = np.array([1000000, 10000000])
            robot_centroid_index = [-1, -1]

            # Current Robot position as recieved from the message (meters in map frame)
            robot_position = np.array([goal[0], goal[1]])

            # Looping through all the centroids and their position in meters, and finding the one that is closest to the respective robot position.
            for centroid_index, centroid_position in temp_centroid_dict.items():
                if np.linalg.norm(robot_position - centroid_position) < np.linalg.norm(robot_position - robot_centroid):
                    robot_centroid = centroid_position
                    robot_centroid_index = [float(x) for x in centroid_index.split(', ')]

            # Assign the closest centroid to that robot in the robot_centroid_dict and delete the centroid from the centroid_dict. 
            # This is so that no 2 robots are assigned to the same centroid.
            id = int(robot.split('_')[1])
            self.robots_goal_centroids[id] = robot_centroid_index
            removeal_key = list(temp_centroid_dict.keys())[list(temp_centroid_dict.values()).index(robot_centroid)]
            del temp_centroid_dict[removeal_key]

    def update_robot_centroids(self):
        '''
        This function updates the robot_centroids dictionary. With the current values in self.robot_states.
        '''

        temp_centroid_dict = self.centroid_dict.copy()

        for robot, state in self.robot_states.items():
            # starting values that are so unrealistic that they will be overwritten every time
            robot_centroid = np.array([1000000, 10000000])
            robot_centroid_index = [-1, -1]

            # Current Robot position as recieved from the message (meters in map frame)
            robot_position = np.array([state.pose.pose.position.x, state.pose.pose.position.y])

            # Looping through all the centroids and their position in meters, and finding the one that is closest to the respective robot position.
            for centroid_index, centroid_position in temp_centroid_dict.items():
                if np.linalg.norm(robot_position - centroid_position) < np.linalg.norm(robot_position - robot_centroid):
                    robot_centroid = centroid_position
                    robot_centroid_index = [float(x) for x in centroid_index.split(', ')]

            # Assign the closest centroid to that robot in the robot_centroid_dict and delete the centroid from the centroid_dict. 
            # This is so that no 2 robots are assigned to the same centroid.
            id = int(robot.split('_')[1])
            self.robot_centroids[id] = robot_centroid_index
            removeal_key = list(temp_centroid_dict.keys())[list(temp_centroid_dict.values()).index(robot_centroid)]
            del temp_centroid_dict[removeal_key]

    def state_subscriber_callback(self, msg, robot_name):
        self.robot_states[robot_name] = msg
    
    def odom_subscriber_callback(self, msg, robot_name):
        time_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9

        linear_velocity_msg = msg.twist.twist.linear
        linear_velocity = (linear_velocity_msg.x, linear_velocity_msg.y)

        angular_velocity_msg = msg.twist.twist.angular
        angular_velocity = (angular_velocity_msg.x, angular_velocity_msg.y, angular_velocity_msg.z)
        id = int(robot_name.split('_')[1])
        self.result_dict['odometry'][id].append({'time_stamp': time_stamp, 'linear_vel': linear_velocity, 'angular_vel': angular_velocity})

    def laser_subscriber_callback(self, msg, robot_name):
        time_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9

        min_range = min(msg.ranges)
        id = int(robot_name.split('_')[1])
        self.result_dict['min_range'][id].append({'time_stamp': time_stamp, 'min_range': min_range})
    
    def update_next_goal(self):
        for robot_name, robot_goal in self.robots_goal_dict.items():
            self.next_goals[robot_name] = robot_goal

    def send_mapf_request(self):

        # Translating the latest robot position into the respective centroid position
        self.update_robot_centroids()

        # Creating the request object
        request = RemrocMapfsolver.Request()

        # For every robot
        for id, goal_centroid in self.robots_goal_centroids.items():
            
            temp_msg = RemrocPose2darray()

            # Put the starting centroids into a Pose2D array
            start_pose = Pose2D()
            start_pose.x = self.robot_centroids[id][0]
            start_pose.y = self.robot_centroids[id][1]

            # Put the goal centroids into a Pose2D array
            goal_pose = Pose2D()
            goal_pose.x = goal_centroid[0]
            goal_pose.y = goal_centroid[1]

            temp_msg.id = id

            temp_msg.poses.append(start_pose)
            temp_msg.poses.append(goal_pose)

            # Add the respective Robot start and goal positions to the request object
            request.poses.append(temp_msg)

        # Make the service requ0est and recieve response
        # self.get_logger().info(f'{request}')
        self.future = self.cli.call_async(request)
        self.get_logger().info('Request made.')
        rclpy.spin_until_future_complete(self, self.future)

        # Saving the latest mapf solution.
        for array in self.future.result().robots_waypoint_array:
            key = array.id
            self.latest_mapf_solution[key] = array.poses
            # self.get_logger().info(f'{array}')

        return self.future.result()
    
    def send_action_goal(self, mapf_response):

        # Translate the latest robot position into centroids.
        # The robot may have moved substantially between the request and return of the mapf solution.
        self.update_robot_centroids()

        # extract the mapf solution into a sequence of cartesian x and y coordinates
        robots_array_dict = {}
        for array in mapf_response.robots_waypoint_array:
            robots_array_dict[array.id] = []
            for pose in array.poses:
                robots_array_dict[array.id].append(self.centroid_dict[f'{int(pose.x)}, {int(pose.y)}'])

        # Check if all robots are still within the first gridsquare of the passed mapf-strategy. 
        # This will NOT be the case if the computation time for the mapf-solution is too big
        valid_mapf_plan = True
        for robot_id, robot_centroid in self.robot_centroids.items():
            # self.get_logger.info(robot_name)
            # Grabbing the first centroid in the mapf_solution of the respective robot
            first_centroid = self.latest_mapf_solution[robot_id][0]
            # self.get_logger().info(f'\n {first_centroid}\n')
            first_centroid_list = [first_centroid.x, first_centroid.y]

            # Check if robot centroid is the same as the first one in the mapf plan
            if first_centroid_list != robot_centroid:
                valid_mapf_plan = False            

        if valid_mapf_plan:
            # Loop through the action_client_dict and set the action server goals
            for robot_name, action_client in self.action_client_dict.items():
                goal_msg = NavigateThroughPoses.Goal()
                
                for pose in robots_array_dict[int(robot_name.split('_')[1])]:
                    pose_stamped = PoseStamped()
                    # TODO: add the timeaspect to the pose
                    pose_stamped.header.frame_id = 'map'
                    pose_stamped.pose.position.x = pose[0]
                    pose_stamped.pose.position.y = pose[1]
                    goal_msg.poses.append(pose_stamped)

                for i, pose_stamped in enumerate(goal_msg.poses[:-1]):
                    p1 = np.array([pose_stamped.pose.position.x, pose_stamped.pose.position.y])
                    p2 = np.array([goal_msg.poses[i+1].pose.position.x, goal_msg.poses[i+1].pose.position.y])

                    #skip repearing poses.
                    temp_i = i
                    while (p1 == p2).all():
                        temp_i += 1
                        p2 = np.array([goal_msg.poses[temp_i+1].pose.position.x, goal_msg.poses[temp_i+1].pose.position.y])

                    angle = (np.arctan2(p2[0]- p1[0], p2[1]- p1[1]) - np.pi/2) % 2*np.pi

                    rotation = Rotation.from_euler('z', angle)

                    quaternion = rotation.as_quat()

                    pose_stamped.pose.orientation.x = quaternion[0]
                    pose_stamped.pose.orientation.y = quaternion[1]
                    pose_stamped.pose.orientation.z = quaternion[2]
                    pose_stamped.pose.orientation.w = quaternion[3]
                
                # This look looks for a "wait" command aka. two following poses that are identical. 
                # If it finds them, it removes all the following goal states, to make the robot wait in the position.
                for i, pose_stamped in enumerate(goal_msg.poses[:-1]):
                    pose_x = pose_stamped.pose.position.x
                    pose_y = pose_stamped.pose.position.y
                    future_pose_x = goal_msg.poses[i+1].pose.position.x
                    future_pose_y = goal_msg.poses[i+1].pose.position.y
                    if pose_x == future_pose_x and pose_y == future_pose_y:
                        del goal_msg.poses[i+1:]
                        break

                action_client.wait_for_server()
                action_client.send_goal_async(goal_msg)

        # If there is no valid mapf_solution, e.g. the solver took to long and the robots have moved on, send a stop command
        else:
            for robot_name, cmd_vel_publisher in self.cmd_vel_pub_dict.items():
                velocity_command = Twist()
                cmd_vel_publisher.publish(velocity_command)
            self.get_logger().info('Invalid mapf solution. Stoping the robots. --------!!!')
        

    def check_robots_at_goal(self) -> bool:
        '''
        Checks wether the robots are at the goal state and if so collect metrics. 
        '''
        self.node_current_time = self.get_clock().now()
        robots_at_goal = True
        for robot_id, robot_centroid in self.robot_centroids.items():
            # check 3 criteria: 
            # 1. robot at goal_centroid
            # 2. no entry in the time_to_goal metric yet
            # 3. mapf_solution length == 1
            if (self.robots_goal_centroids[robot_id] == robot_centroid):
                if (self.result_dict['time_to_goal'][robot_id] == None) and (len(self.latest_mapf_solution[robot_id]) == 1):
                    time_msg = (self.node_current_time - self.node_start_time).to_msg()
                    self.result_dict['time_to_goal'][robot_id] = time_msg.sec + time_msg.nanosec*1e-9
                    # print(self.result_dict)
            else:
                robots_at_goal = False

        return robots_at_goal
            
def main():
    rclpy.init()

    experiment = ExperimentAsync()

    experiment.node_start_time = experiment.get_clock().now()

    loop_start_time = experiment.get_clock().now()

    # Loop untill all robots have reached their goal-state
    robots_at_goal = False
    while not robots_at_goal:
        # checks wether the preceeding loop has not taken too long
        if (experiment.get_clock().now()-loop_start_time >= experiment.target_coordinator_loop_time):
            rclpy.spin_once(experiment)
            loop_start_time = experiment.get_clock().now()

            # recieve all robots current state
            all_states_recieved = False
            while not all_states_recieved:
                rclpy.spin_once(experiment)
                all_states_recieved=True
                for robot_state in experiment.robot_states.values():
                    if robot_state is None: all_states_recieved = False

            # query the mapf solver with an up-to-date mapf instance
            response = experiment.send_mapf_request()
            experiment.get_logger().info(f'Response recieved.')
            for x in response.robots_waypoint_array:
                experiment.get_logger().info(f'{experiment.robot_centroids[x.id]}')
                experiment.get_logger().info(f'{x.poses[0]}')
            # experiment.get_logger().info(f'{response}')
            
            # Compare robots current position to the position of their starting state.
            robots_at_goal = experiment.check_robots_at_goal()

            time_msg = (experiment.node_current_time - experiment.node_start_time).to_msg()
            time_sec = time_msg.sec + time_msg.nanosec*1e-9
            if time_sec >= 600:
                robots_at_goal = True

            # Take the response and use it in an action clients that make request to the respective nav2 "navigate_through_pose" action servers
            experiment.send_action_goal(response)
            experiment.get_logger().info(f'Goal send.')

            # Reset the robot_states dictionary for the next loop
            experiment.robot_states = {robot_name: None for robot_name in experiment.robot_names}

            rclpy.spin_once(experiment)
            loop_end_time = experiment.get_clock().now()

            # Write loop time
            time_msg = (loop_end_time-loop_start_time).to_msg()
            experiment.result_dict['coordinator_loop_time'].append(time_msg.sec + time_msg.nanosec*1e-9)

        else:
            rclpy.spin_once(experiment)
    
    # write the experiment results into a json.
    target_path = Path(f"results/mapf/{experiment.map_name}/{experiment.number_of_humans}/{experiment.sample}.json")
    target_path.parent.mkdir(exist_ok=True, parents=True)
    with target_path.open('w') as result_file:
        json.dump(experiment.result_dict, result_file, indent=4)

    experiment.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()