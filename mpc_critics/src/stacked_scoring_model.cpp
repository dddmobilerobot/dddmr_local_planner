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
#include <mpc_critics/stacked_scoring_model.h>

namespace mpc_critics
{


StackedScoringModel::StackedScoringModel(
                              const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& m_logger,
                              std::shared_ptr<tf2_ros::Buffer> m_tf2Buffer)
{
  logger_ = m_logger;
  shared_data_ = std::make_shared<mpc_critics::ModelSharedData>(m_tf2Buffer);
  access_ = new model_mutex_t();
  return;
}

StackedScoringModel::~StackedScoringModel()
{
  shared_data_.reset();
  delete access_;
}

/*
Plugin operations
*/

void StackedScoringModel::addPluginByTraj(std::string traj_name, std::shared_ptr<ScoringModel> model)
{

  /*This is very important line that assign pointer to each model for shared data*/
  model->setSharedData(shared_data_);

  if ( models_map_.find(traj_name) == models_map_.end() ) {
    // not found
    std::vector<std::shared_ptr<mpc_critics::ScoringModel>> models;
    models_map_.insert({traj_name, models});
    models_map_[traj_name].push_back(model);
  } else {
    // found
    models_map_[traj_name].push_back(model);
  }

}

void StackedScoringModel::scoreTrajectory(std::string traj_gen_name, base_trajectory::Trajectory& one_traj){

  for (std::vector<std::shared_ptr<ScoringModel> >::iterator model =  models_map_[traj_gen_name].begin(); model !=  models_map_[traj_gen_name].end();
       ++model)
  {
    
    RCLCPP_DEBUG(logger_->get_logger(), "Rate trajectory by critics: %s", (*model)->getModelName().c_str());
    double return_cost = (*model)->scoreTrajectory(one_traj);
    if(return_cost<0){
      one_traj.cost_ = return_cost;
      break;
    }
    else{
      one_traj.cost_ += return_cost;
    }

  }

}




}//end of name space