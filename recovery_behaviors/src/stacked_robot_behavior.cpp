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
#include <recovery_behaviors/stacked_robot_behavior.h>

namespace recovery_behaviors
{


StackedRobotBehavior::StackedRobotBehavior(
                      const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& m_logger,
                      std::shared_ptr<tf2_ros::Buffer> m_tf2Buffer)
{
  logger_ = m_logger;
  shared_data_ = std::make_shared<recovery_behaviors::RecoveryBehaviorsSharedData>(m_tf2Buffer);
  access_ = new behavior_mutex_t();
  return;
}

StackedRobotBehavior::~StackedRobotBehavior()
{
  shared_data_.reset();
  delete access_;
}

/*
Plugin operations
*/
void StackedRobotBehavior::addPlugin(std::string behavior_name, std::shared_ptr<RobotBehavior> behavior)
{

  /*This is very important line that assign pointer to each model for shared data*/
  behavior->setSharedData(shared_data_);

  /*This is very important line that assign pointer to each behavior for shared data*/
  //model->setSharedData(shared_data_);
  robot_behaviors_.insert({behavior_name, behavior});
}

dddmr_sys_core::RecoveryState StackedRobotBehavior::runBehavior( 
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::RecoveryBehaviors>> goal_handle)
{
  
  const auto goal = goal_handle->get_goal();
  std::string behavior_name = goal->behavior_name;
  if (robot_behaviors_.find(behavior_name) == robot_behaviors_.end()){
    RCLCPP_ERROR(logger_->get_logger(), "%s not presented.", behavior_name.c_str());
    return dddmr_sys_core::RecoveryState::RECOVERY_BEHAVIOR_NOT_FOUND;
  }
  return robot_behaviors_[behavior_name]->runBehavior(goal_handle);

}

}//end of name space