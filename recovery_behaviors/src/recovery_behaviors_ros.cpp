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
#include <recovery_behaviors/recovery_behaviors_ros.h>

namespace recovery_behaviors
{

Recovery_Behaviors_ROS::Recovery_Behaviors_ROS(const std::string& name,
      std::shared_ptr<perception_3d::Perception3D_ROS> perception_3d,
      std::shared_ptr<mpc_critics::MPC_Critics_ROS> mpc_critics,
      std::shared_ptr<trajectory_generators::Trajectory_Generators_ROS> trajectory_generators):
  Node(name),
  name_(name),
  stacked_robot_behavior_(NULL),
  behavior_loader_(name, "recovery_behaviors::RobotBehavior"),
  perception_3d_ros_(perception_3d),
  mpc_critics_ros_(mpc_critics),
  trajectory_generators_ros_(trajectory_generators)
{
}

rclcpp_action::GoalResponse Recovery_Behaviors_ROS::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const dddmr_sys_core::action::RecoveryBehaviors::Goal> goal)
{
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Recovery_Behaviors_ROS::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::RecoveryBehaviors>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void Recovery_Behaviors_ROS::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::RecoveryBehaviors>> goal_handle)
{

  if (is_active(current_handle_)){
    RCLCPP_INFO(this->get_logger(), "An older goal is active, cancelling current one.");
    auto result = std::make_shared<dddmr_sys_core::action::RecoveryBehaviors::Result>();
    current_handle_->abort(result);
    return;
  }
  else{
    current_handle_ = goal_handle;
  }
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&Recovery_Behaviors_ROS::runBehavior, this, std::placeholders::_1), goal_handle}.detach();
}

void Recovery_Behaviors_ROS::initial(){

  this->declare_parameter("odom_topic", rclcpp::ParameterValue("odom"));
  this->get_parameter("odom_topic", odom_topic_);
  RCLCPP_INFO(this->get_logger(), "odom_topic: %s", odom_topic_.c_str());

  //@Initialize transform listener and broadcaster
  tf_listener_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  tf2Buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface(),
    tf_listener_group_);
  tf2Buffer_->setCreateTimerInterface(timer_interface);
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer_);

  cbs_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cbs_group_;

  odom_ros_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 2,
      std::bind(&Recovery_Behaviors_ROS::cbOdom, this, std::placeholders::_1), sub_options);

  /*
  Start to load plugins, the shared data is also initialized when new
  */
  stacked_robot_behavior_ = new StackedRobotBehavior(this->get_node_logging_interface(), tf2Buffer_);

  //@Start to load plugins
  this->declare_parameter("plugins", rclcpp::PARAMETER_STRING_ARRAY);
  rclcpp::Parameter plugins = this->get_parameter("plugins");
  plugins_ = plugins.as_string_array();
  for(auto i=plugins_.begin(); i!=plugins_.end(); i++){

    //@ get plugin type
    std::string get_plugin_type_str = (*i)+".plugin"; //plugins.plugin, ex: map.plugin
    this->declare_parameter(get_plugin_type_str, rclcpp::ParameterValue(""));
    rclcpp::Parameter plugin_type_param = this->get_parameter(get_plugin_type_str);
    std::string plugin_type_str = plugin_type_param.as_string();

    RCLCPP_INFO(this->get_logger(), "Use behavior name: %s ---> %s", (*i).c_str(), plugin_type_str.c_str());  

    std::shared_ptr<recovery_behaviors::RobotBehavior> plugin = behavior_loader_.createSharedInstance(plugin_type_str);
    stacked_robot_behavior_->addPlugin((*i), plugin);
    //Pass shared data for Sensor to share customized data betrween plugins
    plugin->initialize((*i), shared_from_this(), perception_3d_ros_, mpc_critics_ros_, trajectory_generators_ros_);
  }

  //@Create action server
  action_server_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  this->action_server_recovery_behaviors_ = rclcpp_action::create_server<dddmr_sys_core::action::RecoveryBehaviors>(
    this,
    "/recovery_behaviors",
    std::bind(&Recovery_Behaviors_ROS::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&Recovery_Behaviors_ROS::handle_cancel, this, std::placeholders::_1),
    std::bind(&Recovery_Behaviors_ROS::handle_accepted, this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    action_server_group_);
}

Recovery_Behaviors_ROS::~Recovery_Behaviors_ROS()
{
  delete stacked_robot_behavior_;
}

void Recovery_Behaviors_ROS::cbOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Got odom.");
  stacked_robot_behavior_->getSharedDataPtr()->robot_state_ = *msg;
}

dddmr_sys_core::RecoveryState Recovery_Behaviors_ROS::runBehavior(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddmr_sys_core::action::RecoveryBehaviors>> goal_handle)
{
  return stacked_robot_behavior_->runBehavior(goal_handle);
}

}//end of name space