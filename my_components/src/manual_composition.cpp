#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "my_components/approach_service_server.hpp"


int main(int argc, char *argv[]){
    
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::NodeOptions options;
    auto node = std::make_shared<my_components::AttachServer>(options);
    executor.add_node(node);
    executor.spin();


    rclcpp::shutdown();
    return 0;
}