#ifndef COMPOSITION__ATTACH_CLIENT_HPP
#define COMPOSITION__ATTACH_CLIENT_HPP

#include "my_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/go_to_loading.hpp"

namespace my_components {

class AttachClient : public rclcpp::Node{
    
    public:
        COMPOSITION_PUBLIC
        explicit AttachClient(const rclcpp::NodeOptions &options);

    private:

        rclcpp::Client<custom_interfaces::srv::GoToLoading>::SharedPtr client;

        void service_response(rclcpp::Client<custom_interfaces::srv::GoToLoading>::SharedFuture fut);
};


} // namespace my_components

#endif