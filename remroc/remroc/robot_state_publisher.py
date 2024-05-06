
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
from rclpy.node import Node

from nav_msgs.msg import Odometry
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_geometry_msgs import PoseStamped


class RobotStatePublisher(Node):
    '''
    This Node subscribes the tf tree in its respective namespace and uses the 
    map -> odom transformation published by a localization algorithm like amcl.
    It subscripes to the "/odometry/filtered" topic which is published by a 
    state-estimation algorithm like an ekf. It then uses the afore mentioned transform 
    to publish the same message only in the map frame. 
    '''
    def __init__(self):
        super().__init__('robot_state_publisher')

        # The target frame to which the messages should be transformed
        self.target_frame = 'map'

        # Creating the tf_listener to get the transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Creating the publisher and subscriber to the respective topics
        self.publisher_ = self.create_publisher(Odometry, 'robot_state', 10)
        self.subscriber_ = self.create_subscription(Odometry, 'odometry/filtered', self.callback_function, 10)

    def callback_function(self, msg):
        
        response = msg

        # Creating a PoseStamped message, as from the Odometry, only the Pose needs to be transformed
        src_pose = PoseStamped()
        src_pose.header = msg.header
        src_pose.pose = msg.pose.pose
        
        # Trying to get the current transform and transform the pose. 
        try:
            transformed_pose = self.tf_buffer.transform(
                src_pose, 
                target_frame=self.target_frame, 
                )
        # In case no transform is available print exeption and finish the callback
        except TransformException as ex:
            self.get_logger().info(f'Could not transform: {ex}')
            return
        
        # If the transformation was sucessfull, fill the response and publish it
        response.header = transformed_pose.header
        response.pose.pose = transformed_pose.pose

        self.publisher_.publish(response)

def main(args=None):
    rclpy.init(args=args)

    human_pose_publisher = RobotStatePublisher()

    rclpy.spin(human_pose_publisher)

    human_pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()