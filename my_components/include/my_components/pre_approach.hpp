#ifndef COMPOSITION__PRE_APPROACH_HPP_
#define COMPOSITION__PRE_APPROACH_HPP_

#include "my_components/visibility_control.h"
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

class PreApproach : public rclcpp::Node{

    public:
        COMPOSITION_PUBLIC
        explicit PreApproach(const rclcpp::NodeOptions & options);

    private:
        float obstacle = 0.6;
        int degrees = -90;
        geometry_msgs::msg::Twist vel;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
        bool turn;  
        bool tu = true;
        bool send = true;  

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

        float euler_degree_transform(const nav_msgs::msg::Odometry::SharedPtr msg);
};

}  // namespace composition

#endif 