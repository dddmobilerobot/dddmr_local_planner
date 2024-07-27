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
#ifndef MPC_CRITICS_ROS_H_
#define MPC_CRITICS_ROS_H_

#include <mpc_critics/stacked_scoring_model.h>

namespace mpc_critics
{

class MPC_Critics_ROS : public rclcpp::Node {
  
  public:

    MPC_Critics_ROS(const std::string& name);
    ~MPC_Critics_ROS();
    
    void initial();

    void updateSharedData(); 
    
    void scoreTrajectory(std::string traj_gen_name, base_trajectory::Trajectory& one_traj);

    StackedScoringModel* getStackedScoringModelPtr(){return stacked_scoring_model_;}  

    std::shared_ptr<mpc_critics::ModelSharedData> getSharedDataPtr(){return stacked_scoring_model_->getSharedDataPtr();}

  private:
    
    rclcpp::CallbackGroup::SharedPtr tf_listener_group_;
    /*For plugin loader*/
    pluginlib::ClassLoader<mpc_critics::ScoringModel> model_loader_;
    std::vector<std::string> plugins_;
    
  protected:

    StackedScoringModel* stacked_scoring_model_;

    std::shared_ptr<tf2_ros::TransformListener> tfl_;
    std::shared_ptr<tf2_ros::Buffer> tf2Buffer_;  ///< @brief Used for transforming point clouds
    std::string name_;


};

}//end of name space
#endif