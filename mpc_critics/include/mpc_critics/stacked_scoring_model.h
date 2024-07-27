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
#ifndef MPC_CRITICS_STACKED_SCORING_H_
#define MPC_CRITICS_STACKED_SCORING_H_

#include <mpc_critics/scoring_model.h>
/*Debug*/
#include <sys/time.h>
#include <time.h>

/*To iterate the ymal for plugins*/
#include <pluginlib/class_loader.hpp>

namespace mpc_critics
{

class StackedScoringModel{

  public:

    StackedScoringModel(const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& m_logger, 
                        std::shared_ptr<tf2_ros::Buffer> m_tf2Buffer);
    ~StackedScoringModel();

    /*Plugin operations*/

    void addPluginByTraj(std::string traj_name, std::shared_ptr<ScoringModel> model);

    std::shared_ptr<mpc_critics::ModelSharedData> getSharedDataPtr(){return shared_data_;}

    void scoreTrajectory(std::string traj_gen_name, base_trajectory::Trajectory& one_traj);
    
    // Provide a typedef to ease future code maintenance
    typedef std::recursive_mutex model_mutex_t;
    model_mutex_t* getMutex()
    {
      return access_;
    }

  private:

    std::map<std::string, std::vector<std::shared_ptr<mpc_critics::ScoringModel>>> models_map_;
    std::shared_ptr<mpc_critics::ModelSharedData> shared_data_;
    /*mutex*/
    model_mutex_t* access_;

    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_;

};

}//end of name space
#endif