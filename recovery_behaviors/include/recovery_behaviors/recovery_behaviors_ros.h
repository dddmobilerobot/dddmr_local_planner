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

class Recovery_Behaviors_ROS : public rclcpp::Node {
  
  public:
    Recovery_Behaviors_ROS(const std::string& name, 
        std::shared_ptr<perception_3d::Perception3D_ROS> perception_3d,
        std::shared_ptr<mpc_critics::MPC_Critics_ROS> mpc_ctitics,
        std::shared_ptr<trajectory_generators::Trajectory_Generators_ROS> trajectory_generators);
    ~Recovery_Behaviors_ROS();

    dddmr_sys_core::RecoveryState runBehavior(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::RecoveryBehaviors>> goal_handle);
    
    void initial();

  private:

    rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const dddmr_sys_core::action::RecoveryBehaviors::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::RecoveryBehaviors>> goal_handle);

    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::RecoveryBehaviors>> goal_handle);
    
    rclcpp_action::Server<dddmr_sys_core::action::RecoveryBehaviors>::SharedPtr action_server_recovery_behaviors_;
    
    std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::RecoveryBehaviors>> current_handle_;

    /*Sub*/
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_ros_sub_;
    
    rclcpp::CallbackGroup::SharedPtr action_server_group_;
    rclcpp::CallbackGroup::SharedPtr tf_listener_group_;
    rclcpp::CallbackGroup::SharedPtr cbs_group_;

    /*For plugin loader*/
    pluginlib::ClassLoader<recovery_behaviors::RobotBehavior> behavior_loader_;
    
    std::vector<std::string> plugins_;

    bool is_active(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::RecoveryBehaviors>> handle) const
    {
      return handle != nullptr && handle->is_active();
    }


  protected:

    StackedRobotBehavior* stacked_robot_behavior_;

    std::shared_ptr<tf2_ros::TransformListener> tfl_;
    std::shared_ptr<tf2_ros::Buffer> tf2Buffer_;  ///< @brief Used for transforming point clouds
    std::string name_;

    //@ for percertion
    std::shared_ptr<perception_3d::Perception3D_ROS> perception_3d_ros_;
    //@ for critics
    std::shared_ptr<mpc_critics::MPC_Critics_ROS> mpc_critics_ros_;
    //@ for trajectory generator
    std::shared_ptr<trajectory_generators::Trajectory_Generators_ROS> trajectory_generators_ros_;

    //@for odom subscriber
    std::string odom_topic_;
    void cbOdom(const nav_msgs::msg::Odometry::SharedPtr msg);

};

}//end of name space
