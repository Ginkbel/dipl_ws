#include "nmcp_navigation.hpp"

using std::placeholders::_1;

NMPCControllerROS::NMPCControllerROS() : Node("nmpc_astro"), mpc_iter(0)
{

  control_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 1, std::bind(&NMPCControllerROS::setGoal, this, _1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1, std::bind(&NMPCControllerROS::setCurrentState, this, _1));

  timer1_ = this->create_wall_timer(16.667ms, std::bind(&NMPCControllerROS::controlLoop, this)); // rate is 60Hz (16.667ms)

  is_goal_set = false;
  nmpc_.setUp();

  x_ref = {0.0, 0.0, 0.0};
  nmpc_.setReference(x_ref);

}

void NMPCControllerROS::setGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
  tf2::Matrix3x3 m(q);
  m.getRPY(roll_, pitch_, yaw_);

  RCLCPP_INFO(this->get_logger(),"setGoal called!");


  // ROS info goal
  // RCLCPP_INFO(this->get_logger(), "Goal x: %f, y: %f, theta: %f", msg->pose.position.x, msg->pose.position.y, yaw_);
  x_ref = {msg->pose.position.x, msg->pose.position.y, yaw_};
  nmpc_.setReference(x_ref);
  is_goal_set = true;
}

void NMPCControllerROS::controlLoop()
{
  std::vector<double> u0 = {0.0, 0.0};
  u_opt = (!x0.empty() && is_goal_set) ? nmpc_.solve(x0) : u0;
  // u_opt = u0;

  if (is_goal_set && !x0.empty())
  {
    double dist = sqrt(pow(x0[0] - x_ref[0], 2) + pow(x0[1] - x_ref[1], 2) +  pow(x0[2] - x_ref[2], 2));
    // TODO: terminal cost pa da ne treba taj clan?
    if (dist < 0.01)
    {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      is_goal_set = false;
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.linear.z = 0.0;
      cmd_vel.angular.z = 0.0;
      control_pub_->publish(cmd_vel);
      // mpc_iter++;
    }
    else
    {
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = u_opt[0];
      cmd_vel.linear.y = 0.0;
      cmd_vel.linear.z = 0.0;
      cmd_vel.angular.z = u_opt[1];
      control_pub_->publish(cmd_vel);
      // mpc_iter++;
    }
  }
  // RCLCPP_INFO(this->get_logger(),"Control opt: v: %f, w: %f", u_opt[0], u_opt[1]);

}

void NMPCControllerROS::setCurrentState(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  m.getRPY(roll_, pitch_, yaw_);

  // set state
  x0 = {msg->pose.pose.position.x, msg->pose.pose.position.y, yaw_};

  // RCLCPP_INFO(this->get_logger(), "Current state: x: %f, y: %f, theta: %f", x0[0], x0[1], yaw_);

}

