
// Copyright (c) 2024 - for information on the respective copyright owner
// see the NOTICE file or the repository https://github.com/boschresearch/remroc/.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <cmath>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "remroc_interfaces/srv/remroc_mapfsolver.hpp"
#include "remroc_interfaces/msg/remroc_pose2darray.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "remroc_cbs_library/cbs.hpp"

#include <memory>

class RemrocMapfSolverServiceServer : public rclcpp::Node
{
  public: 
    RemrocMapfSolverServiceServer() : Node("mapf_solver_server")
    {
      using namespace std::placeholders;
      this -> declare_parameter("mapf_file_path", ".");
      srv_ = create_service<remroc_interfaces::srv::RemrocMapfsolver>("mapf_solver", std::bind(&RemrocMapfSolverServiceServer::solve, this, _1, _2));
    }

  private: 
    rclcpp::Service<remroc_interfaces::srv::RemrocMapfsolver>::SharedPtr srv_;

    void solve(std::shared_ptr<remroc_interfaces::srv::RemrocMapfsolver::Request> request,
              std::shared_ptr<remroc_interfaces::srv::RemrocMapfsolver::Response> response)
    {
      // Define the input vectors

      std::unordered_map<int, std::vector<std::vector<int>>> input_map;
      std::vector<std::vector<int>> robot_vector;
      std::string mapf_file_path = this->get_parameter("mapf_file_path").as_string();

      int my_x;
      int my_y;
      int id;

      for (const auto& pose_vec : request->poses) {
        id = pose_vec.id;
        
        for (const auto& pose : pose_vec.poses) {
          my_x = std::round(pose.x);
          my_y = std::round(pose.y);
          robot_vector.push_back({my_x, my_y});
        }

        input_map[id] = robot_vector;
        robot_vector.clear();
      }

      std::unordered_map<int, std::vector<std::vector<int>>> my_robots_waypoint_array = remroc_cbs_library::get_robots_waypoint_array(mapf_file_path, input_map);

      remroc_interfaces::msg::RemrocPose2darray robot_array;

      geometry_msgs::msg::Pose2D point;

      for (auto it = my_robots_waypoint_array.begin(); it != my_robots_waypoint_array.end(); ++it) {
        robot_array.id = it->first;
        
        for (const auto& pose : it->second) {
          point.x = pose[0];
          point.y = pose[1];
          robot_array.poses.emplace_back(point);
        }

        response->robots_waypoint_array.emplace_back(robot_array);
        robot_array.poses = {};
      }
    }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = std::make_shared<RemrocMapfSolverServiceServer>();
  
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Launched MAPF solver server for the simple environment.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}