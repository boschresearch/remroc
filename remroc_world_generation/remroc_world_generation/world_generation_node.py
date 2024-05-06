
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
from rclpy.node import Node
from pathlib import Path
import numpy as np

from ament_index_python.packages import get_package_share_directory

class WorldGenerationNode(Node):

    def __init__(self):
        super().__init__("world_generation_node")
        self.declare_parameter('number_of_humans', 0)
        self.declare_parameter('map', 'simple')
        self.number_of_humans = self.get_parameter('number_of_humans').value
        self.map = self.get_parameter('map').value

        self.remroc_world_generation_pkg_path = get_package_share_directory('remroc_world_generation')
        self.placeholder_file = self.map + '_placeholder_' + str(self.number_of_humans) + '.sdf'
        self.world_file = self.map + '_' + str(self.number_of_humans) + '.sdf'

    def create_world(self):

        with Path(self.remroc_world_generation_pkg_path).joinpath('worlds', 'human_trajectories.json').open('r') as data:
            trajectory_dict = json.load(data)

        human_time_string_dictionary = {x:"" for x in trajectory_dict.keys()}
        for human, waypoints in trajectory_dict.items():
            temp_str = ""
            timespan = np.random.randint(0, 1)
            for time_temp in range(timespan):
                time_factor = 5
                time = int(time_temp)/time_factor
                temp_str += f"<waypoint><time>{time}</time><pose>{waypoints['0'][0]} {waypoints['0'][1]} {1} 0 0 {waypoints['0'][2]}</pose></waypoint>"
            for timestep, waypoint in waypoints.items():
                x = waypoint[0]
                y = waypoint[1]
                z = 1
                theta = waypoint[2]
                time_factor = 5
                time = (timespan + int(timestep))/time_factor
                time_back = (timespan + (int(timestep) + 2*(len(waypoints.keys())- int(timestep))))/time_factor
                temp_str += f"<waypoint><time>{time}</time><pose>{x} {y} {z} 0 0 {theta}</pose></waypoint>"
                temp_str += f"<waypoint><time>{time_back}</time><pose>{x} {y} {z} 0 0 {theta+np.pi}</pose></waypoint>"

            human_time_string_dictionary[human] = temp_str

        with Path(self.remroc_world_generation_pkg_path).joinpath('worlds', 'sdfs', self.placeholder_file).open('r') as data:
            str_world = data.read()

        for human, waypoints_string in human_time_string_dictionary.items():
            str_world = str_world.replace(f"TRAJECTORY_PLACEHOLDER{human}_", waypoints_string)

        with Path(self.remroc_world_generation_pkg_path).joinpath('worlds', 'sdfs', self.world_file).open('w') as file:
            file.write(str_world)

def main(args=None):
    rclpy.init(args=args)
    world_generation_node = WorldGenerationNode()
    world_generation_node.create_world()
    world_generation_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
