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
#include <mpc_critics/shortest_angle_model.h>

PLUGINLIB_EXPORT_CLASS(mpc_critics::ShortestAngleModel, mpc_critics::ScoringModel)

namespace mpc_critics
{

ShortestAngleModel::ShortestAngleModel(){
  return;
  
}

void ShortestAngleModel::onInitialize(){

  node_->declare_parameter(name_ + ".weight", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".weight", weight_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "weight: %.2f", weight_);

}

double ShortestAngleModel::scoreTrajectory(base_trajectory::Trajectory &traj){

  RCLCPP_DEBUG(node_->get_logger().get_child(name_), "Heading deviation: %.2f", shared_data_->heading_deviation_);
  double weight;
  if(shared_data_->heading_deviation_>=0){
    if(traj.thetav_>=0)
      weight = weight_;
    else
      weight = weight_ * 2;
  }
  else{
    if(traj.thetav_>=0)
      weight = weight_ * 2;
    else
      weight = weight_;  
  }
  RCLCPP_DEBUG(node_->get_logger().get_child(name_), "Heading deviation: %.2f, traj theta_v: %.2f, weight: %.2f", shared_data_->heading_deviation_, traj.thetav_, weight);
  return weight;
}

}//end of name space