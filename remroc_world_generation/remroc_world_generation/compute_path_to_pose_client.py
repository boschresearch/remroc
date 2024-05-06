
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
import rclpy
import json
from rclpy.action import ActionClient
from rclpy.node import Node
from pathlib import Path
import numpy as np
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation as R

from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped

from ament_index_python.packages import get_package_share_directory

class ComputePathToPoseClient(Node):

    def __init__(self):
        super().__init__("compute_path_to_pose_client")
        self.declare_parameter("start", [0., 0., 0.])
        self.declare_parameter("goal", [0., 0., 0.])
        self.declare_parameter("human_id", 0)
        self.start = self.get_parameter("start")
        self.goal = self.get_parameter("goal")
        self.human_id = self.get_parameter("human_id")

        self._action_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.remroc_world_generation_pkg_path = get_package_share_directory('remroc_world_generation')

    def send_goal(self):
        goal_msg = ComputePathToPose.Goal()

        self.get_logger().info(f'start: {self.start.value}')
        self.get_logger().info(f'goal: {self.goal.value}')

        msg_goal = PoseStamped()
        msg_goal.header.frame_id = "map"
        msg_goal.pose.position.x = self.goal.value[0]
        msg_goal.pose.position.y = self.goal.value[1]
        # msg_goal.pose.position.z = 0.
        temp_R = R.from_euler("z", self.goal.value[2], degrees=False)
        msg_goal.pose.orientation.x = temp_R.as_quat()[0]
        msg_goal.pose.orientation.y = temp_R.as_quat()[1]
        msg_goal.pose.orientation.z = temp_R.as_quat()[2]
        msg_goal.pose.orientation.w = temp_R.as_quat()[3]

        msg_start = PoseStamped()
        msg_start.header.frame_id = "map"
        msg_start.pose.position.x = self.start.value[0]
        msg_start.pose.position.y = self.start.value[1]
        # msg_start.pose.position.z = 0.
        temp_R = R.from_euler("z", self.start.value[2], degrees=False)
        msg_start.pose.orientation.x = temp_R.as_quat()[0]
        msg_start.pose.orientation.y = temp_R.as_quat()[1]
        msg_start.pose.orientation.z = temp_R.as_quat()[2]
        msg_start.pose.orientation.w = temp_R.as_quat()[3]

        goal_msg.goal = msg_goal
        goal_msg.start = msg_start
        goal_msg.planner_id = "GridBased"
        goal_msg.use_start = True

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
    
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return
        
        self.get_logger().info(f"Goal accepted :) {self.human_id.value}")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        path = []
        path_dict_count = 0
        path_dict = {}
        for pose_stamped in result.path.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            a = pose_stamped.pose.orientation.x
            b = pose_stamped.pose.orientation.y
            c = pose_stamped.pose.orientation.z
            d = pose_stamped.pose.orientation.w
            temp_R = R.from_quat([a, b, c, d])
            theta = temp_R.as_euler('zyx')[0]
            path.append([x, y, theta])
            path_dict[path_dict_count] = [x, y, theta]
            
            path_dict_count += 1

        trajectory_file = Path(self.remroc_world_generation_pkg_path).joinpath('worlds', 'human_trajectories.json')
        if trajectory_file.is_file():
            with trajectory_file.absolute().open("r") as file:
                trajectory_dict = json.load(file)
            keylist = list(trajectory_dict.keys())
            for key in keylist:
                if int(key) == self.human_id.value:
                    del trajectory_dict[key]
        else:
            trajectory_dict = {}

        with trajectory_file.absolute().open("w") as file:
            trajectory_dict[self.human_id.value] = path_dict
            json.dump(trajectory_dict, file, indent=4)

        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Recieved feedback: {feedback}")

def main(args=None):
    rclpy.init(args=args)
    action_client = ComputePathToPoseClient()
    action_client.send_goal()
    rclpy.spin(action_client)

if __name__ == "__main__":
    main()