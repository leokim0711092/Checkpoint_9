#include "my_components/pre_approach.hpp"

#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <cmath>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <string>

namespace my_components
{

    PreApproach::PreApproach(const rclcpp::NodeOptions & options): Node("pre_approach", options)
    {

        sub_scan = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10 , 
        std::bind(&PreApproach::scan_callback, this, std::placeholders::_1));

        sub_odom = this->create_subscription<nav_msgs::msg::Odometry>("/diffbot_base_controller/odom", 10, 
        std::bind(&PreApproach::odom_callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 10);

    }


    void PreApproach::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg){

        if(msg->ranges[359] > obstacle && !turn ){
            vel.linear.x = 0.5;
        }else if (msg->ranges[359] < obstacle){
            vel.linear.x = 0;
            turn = true;
        }

        RCLCPP_INFO(this->get_logger(), "vel.linear: %f", vel.linear.x);

    }

    void PreApproach::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
        float cur_degree = euler_degree_transform(msg)/M_PI*180;
        RCLCPP_INFO(this->get_logger(), "cur_degrees: %f", cur_degree );
        if (std::fabs(degrees - cur_degree) > 1 && turn ) {
            vel.angular.z = std::fabs(degrees - cur_degree) > 5 ? 
            (degrees - cur_degree)/std::fabs(degrees - cur_degree)*0.4 : (degrees - cur_degree)/std::fabs(degrees - cur_degree)*0.2;

            RCLCPP_INFO(this->get_logger(), "vel.angular: %f", vel.angular.z);
            pub_->publish(vel);
        }else{
            RCLCPP_INFO(this->get_logger(), "vel.angular complete");
            vel.angular.z = 0;
            pub_->publish(vel);
        }
    }

    float PreApproach::euler_degree_transform(const nav_msgs::msg::Odometry::SharedPtr msg){
            float x = msg->pose.pose.orientation.x;
            float y = msg->pose.pose.orientation.y;
            float z = msg->pose.pose.orientation.z;
            float w = msg->pose.pose.orientation.w; 

        return atan2(2 * (w * z + x * y),1 - 2 * (y * y + z * z));
    }
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)