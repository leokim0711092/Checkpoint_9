#ifndef COMPOSITION__APPROACH_SERVICE_SERVER_HPP_
#define COMPOSITION__APPROACH_SERVICE_SERVER_HPP_

#include "my_components/visibility_control.h"


#include "rclcpp/node.hpp"
#include "rclcpp/rate.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <climits>
#include <cmath>
#include <cstddef>
#include <math.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <utility>
#include <vector>
#include <algorithm>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "std_msgs/msg/detail/empty__struct.hpp"
#include "std_msgs/msg/detail/string__struct.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "std_msgs/msg/string.hpp"
#include "custom_interfaces/srv/go_to_loading.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>


namespace my_components {

class AttachServer : public rclcpp::Node{
    
    public:
        COMPOSITION_PUBLIC
        explicit AttachServer(const rclcpp::NodeOptions &options);

    private:

        geometry_msgs::msg::Twist vel;
        std_msgs::msg::String ele;

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan;
        rclcpp::Service<custom_interfaces::srv::GoToLoading>::SharedPtr srv_;
        // tf2_ros::TransformBroadcaster broadcaster_{this}; //this will disappear in short time

        tf2_ros::StaticTransformBroadcaster static_broadcaster_{this};
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_elevator;
        rclcpp::CallbackGroup::SharedPtr callback_group_;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        float x1, x2, y1, y2;
        size_t accept_idx_size=0;

        bool ud_date_cart_frame = false;


        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

        void set_map_cart_tf();

        void attach_callback(const std::shared_ptr<custom_interfaces::srv::GoToLoading::Request> req, 
        const std::shared_ptr<custom_interfaces::srv::GoToLoading::Response> res);

        geometry_msgs::msg::TransformStamped broadcast_transform(geometry_msgs::msg::TransformStamped t, float x,float y);

        void move_towards_cart_frame();

};


} // namespace my_components

#endif