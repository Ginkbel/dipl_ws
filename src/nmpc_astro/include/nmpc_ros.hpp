#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_listener.h"

#include <chrono>

#include "nmcp.hpp"

using namespace std::chrono_literals;

class NMPCControllerROS : public rclcpp::Node
{
  public:
    NMPCControllerROS();
  private:
    bool stateOpti;
    int referencePoseOrTrajectory;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr opti_sub_;
    nmpc_controller::NMPCController nmpc_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool is_goal_set;
    bool trajectory_tracking;
    double roll_, pitch_, yaw_;
    std::vector<double> u_opt;
    std::vector<double> x_ref;
    std::vector<double> x0;
    size_t mpc_iter;
    rclcpp::Time present, past;
  	double normalized_time = 0.0;
    void setGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void setCurrentState(const nav_msgs::msg::Odometry::SharedPtr msg);
    void setCurrentStateOpti(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void controlLoop();
};