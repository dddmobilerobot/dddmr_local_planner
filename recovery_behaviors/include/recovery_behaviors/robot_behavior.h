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
/*Debug*/
#include <chrono>

#include <pluginlib/class_list_macros.hpp>
/*path for trajectory*/
#include <base_trajectory/trajectory.h>

/*For perception plugin*/
#include <perception_3d/perception_3d_ros.h>

/*For critics plugin*/
#include <mpc_critics/mpc_critics_ros.h>

/*For trajectory generators plugin*/
#include <trajectory_generators/trajectory_generators_ros.h>

/*For robot state*/
#include <recovery_behaviors/recovery_behaviors_shared_data.h>

/*For action server from move base*/
#include <dddmr_sys_core/dddmr_enum_states.h>

//@pass action server handle for plugin to abort
#include "rclcpp_action/rclcpp_action.hpp"
#include "dddmr_sys_core/action/recovery_behaviors.hpp"


namespace recovery_behaviors
{

class RobotBehavior{

  public:

    RobotBehavior();

    void initialize(const std::string name,
      const rclcpp::Node::WeakPtr& weak_node,
      std::shared_ptr<perception_3d::Perception3D_ROS> perception_3d,
      std::shared_ptr<mpc_critics::MPC_Critics_ROS> mpc_critics,
      std::shared_ptr<trajectory_generators::Trajectory_Generators_ROS> trajectory_generators
      );

    void setSharedData(std::shared_ptr<recovery_behaviors::RecoveryBehaviorsSharedData> shared_data);

    virtual dddmr_sys_core::RecoveryState runBehavior(
          const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::RecoveryBehaviors>> goal_handle) = 0;

  protected:

    virtual void onInitialize() = 0;
    
    rclcpp::Node::SharedPtr node_;

    std::string name_;

    //@ for percertion
    std::shared_ptr<perception_3d::Perception3D_ROS> perception_3d_ros_;
    //@ for critics
    std::shared_ptr<mpc_critics::MPC_Critics_ROS> mpc_critics_ros_;
    //@ for trajectory generator
    std::shared_ptr<trajectory_generators::Trajectory_Generators_ROS> trajectory_generators_ros_;
    
    std::shared_ptr<recovery_behaviors::RecoveryBehaviorsSharedData> shared_data_;

    bool terminate_by_recovery_behavior_ros_;

};


}//end of name space