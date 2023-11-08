#include "my_components/attach_client.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_components {
    
    
    AttachClient::AttachClient(const rclcpp::NodeOptions &options) : Node("AttachClient",options){
        
        client = this->create_client<custom_interfaces::srv::GoToLoading>("/approach_shelf");      

        while (!client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                "Interrupted while waiting for the service. Exiting.");
                return;
            }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "service not available, waiting again...");
        }

        auto request = std::make_shared<custom_interfaces::srv::GoToLoading::Request>();
        request->attach_to_shelf = true;
        client->async_send_request(request, std::bind(&AttachClient::service_response, this, std::placeholders::_1));

    }

    void AttachClient::service_response(rclcpp::Client<custom_interfaces::srv::GoToLoading>::SharedFuture fut){
        auto status = fut.wait_for(std::chrono::seconds(1));
        if(status == std::future_status::ready){
            auto response = fut.get();
            RCLCPP_INFO(this->get_logger(), "Service was called");
            std::string fb = response->complete ? "true" : "false";
            RCLCPP_INFO(this->get_logger(), "Recevied final_approach: %s", fb.c_str());
        }
        else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
        }
    }


}


#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)
