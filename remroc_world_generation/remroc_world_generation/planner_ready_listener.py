
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
from lifecycle_msgs.msg import TransitionEvent



# This node listens to the state of the planner server and, as soon as it is ready, destroys itself. 
# This is used to start a loop in which a planner client node gets launched sequentially to generate multiple paths for humans.
class PlannerReadyListener(Node):

    def __init__(self):
        super().__init__("planner_ready_listener")
        self.subscription = self.create_subscription(
            TransitionEvent,
            "planner_server/transition_event",
            self.listener_callback,
            10)
        self.ready = False
    
    def listener_callback(self,msg):
        if msg.goal_state.label == "active":
            self.ready = True
        
def main(args=None):
    rclpy.init(args=args)

    planner_ready_listener = PlannerReadyListener()

    while planner_ready_listener.ready == False:
        rclpy.spin_once(planner_ready_listener)
    planner_ready_listener.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()