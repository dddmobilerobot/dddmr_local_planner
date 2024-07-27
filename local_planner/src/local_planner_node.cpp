/*
* BSD 3-Clause License

* Copyright (c) 2024, DDDMobileRobot

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:

* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.

* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.

* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.

* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include <memory>
#include <perception_3d/perception_3d_ros.h>
#include <trajectory_generators/trajectory_generators_ros.h>
#include <mpc_critics/mpc_critics_ros.h>
#include <local_planner/local_planner.h>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv){

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto node_tg = std::make_shared<trajectory_generators::Trajectory_Generators_ROS>("trajectory_generators");
  auto node_mc = std::make_shared<mpc_critics::MPC_Critics_ROS>("mpc_critics");
  auto node_p3 = std::make_shared<perception_3d::Perception3D_ROS>("perception_3d_local");
  auto node_lp = std::make_shared<local_planner::Local_Planner>("local_planner");
 
  executor.add_node(node_tg);
  executor.add_node(node_mc);
  executor.add_node(node_p3);
  executor.add_node(node_lp);

  node_tg->initial();
  node_mc->initial();
  node_p3->initial();
  node_lp->initial(node_p3, node_mc, node_tg);
  executor.spin();


  rclcpp::shutdown();

}
