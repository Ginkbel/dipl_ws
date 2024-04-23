#include "nmpc_ros.hpp"

using std::placeholders::_1;

NMPCControllerROS::NMPCControllerROS() : Node("nmpc_astro"), wait_timeout_(300)
{

  this->declare_parameter("stateOpti", false);
  this->declare_parameter("publishPath", false);
  this->declare_parameter("referencePoseOrTrajectory", 0);

  control_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);

  this->get_parameter<bool>("stateOpti", stateOpti);
  this->get_parameter<bool>("publishPath", publishPath);
  this->get_parameter<int>("referencePoseOrTrajectory", referencePoseOrTrajectory);

  if (stateOpti)
  {
    rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos.best_effort();
    qos.durability_volatile();
    qos.keep_last(5);
    opti_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/vrpn_mocap/Astro_Jurica/pose", qos, std::bind(&NMPCControllerROS::setCurrentStateOpti, this, _1));
  } 
  else
  {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1, std::bind(&NMPCControllerROS::setCurrentState, this, _1));
  }
  
  if (referencePoseOrTrajectory == 0)
  {
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 1, std::bind(&NMPCControllerROS::setGoal, this, _1));

  }

  if (publishPath)
  {
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/nmpc_path", 1);
  }

  using rclcpp::contexts::get_global_default_context;

  // On node shutdown sets robot velocity to zero
  get_global_default_context()->add_pre_shutdown_callback(
  [this]() {
    this->timer_->cancel();

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.z = 0.0;
    control_pub_->publish(cmd_vel);

    this->wait_for_all_acked();

  });

  timer_ = this->create_wall_timer(16.667ms, std::bind(&NMPCControllerROS::controlLoop, this)); // rate is 60Hz (16.667ms)

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

  x_ref = {msg->pose.position.x, msg->pose.position.y, yaw_};
  nmpc_.setReference(x_ref);
  is_goal_set = true;
}

void NMPCControllerROS::controlLoop()
{
  std::vector<double> u0 = {0.0, 0.0};
  casadi::DM x_solution_0 = casadi::DM::zeros(3, 40);

  std::pair<std::vector<double>, casadi::DM> empty_solution = {u0, x_solution_0};

  solution = (!x0.empty()) ? nmpc_.solve(x0) : empty_solution;
  
  u_opt = solution.first;

  if (is_goal_set && !x0.empty() && referencePoseOrTrajectory == 0)
  {
    double dist = sqrt(pow(x0[0] - x_ref[0], 2) + pow(x0[1] - x_ref[1], 2) +  pow(x0[2] - x_ref[2], 2));
    // TODO: terminal cost pa da ne treba taj clan?
    if (dist < 0.1)
    {
      RCLCPP_INFO(this->get_logger(), "Goal reached!");
      is_goal_set = false;
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.linear.z = 0.0;
      cmd_vel.angular.z = 0.0;
      control_pub_->publish(cmd_vel);
    }
    else
    {
      geometry_msgs::msg::Twist cmd_vel;
      cmd_vel.linear.x = u_opt[0];
      cmd_vel.linear.y = 0.0;
      cmd_vel.linear.z = 0.0;
      cmd_vel.angular.z = u_opt[1];
      control_pub_->publish(cmd_vel);
    }
  }

  if (!x0.empty() && (referencePoseOrTrajectory == 1 || referencePoseOrTrajectory == 2))
  {
      present = this->get_clock()->now();
      if (past.seconds() == 0)
      {
        past = this->get_clock()->now();
      }
      auto delta_t = (present - past).seconds();

      normalized_time += delta_t;

      if (referencePoseOrTrajectory == 1)
      {
        nmpc_.setReferenceTrajectoryFigure8(normalized_time);
      }
      else if (referencePoseOrTrajectory == 2)
      {
        nmpc_.setReferenceTrajectoryCircular(normalized_time);
      }
      // RCLCPP_INFO(this->get_logger(), "Time [s] %f", normalized_time);

      geometry_msgs::msg::Twist cmd_vel;
      
      cmd_vel.linear.x = u_opt[0];
      cmd_vel.linear.y = 0.0;
      cmd_vel.linear.z = 0.0;
      cmd_vel.angular.z = u_opt[1];
      control_pub_->publish(cmd_vel);

      past = present;
  }

  if (publishPath)
  {
    casadi::DM x_solution = solution.second;
    
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = this->get_clock()->now();
    path_msg.header.frame_id = "odom"; 

    int num_states = x_solution.size2();

    for (int i = 0; i < num_states; i++)
    {
        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = path_msg.header.stamp;
        pose_stamped.header.frame_id = path_msg.header.frame_id;

        // Extract position and orientation from x_solution
        pose_stamped.pose.position.x = static_cast<double>(x_solution(0, i));
        pose_stamped.pose.position.y = static_cast<double>(x_solution(1, i));
        pose_stamped.pose.position.z = 0;

        path_msg.poses.push_back(pose_stamped);
    }
    path_pub_->publish(path_msg);
  }
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

void NMPCControllerROS::setCurrentStateOpti(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  tf2::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
  tf2::Matrix3x3 m(q);
  m.getRPY(roll_, pitch_, yaw_);

  // set state
  x0 = {msg->pose.position.x, msg->pose.position.y, yaw_};

  // RCLCPP_INFO(this->get_logger(), "Current state: x: %f, y: %f, theta: %f", x0[0], x0[1], yaw_);

}

void NMPCControllerROS::wait_for_all_acked()
{
  if (control_pub_->wait_for_all_acked(wait_timeout_)) {
    RCLCPP_INFO(
      this->get_logger(),
      "All subscribers acknowledge messages");
  } else {
    RCLCPP_INFO(
      this->get_logger(),
      "Not all subscribers acknowledge messages during %" PRId64 " ms",
      static_cast<int64_t>(wait_timeout_.count()));
  }
}