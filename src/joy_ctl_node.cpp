#include "joy_ctl.hpp"

int main(int argc, char * argv [])
{
	rclcpp::init(argc, argv); 

	// Create node 
	rclcpp::Node::SharedPtr node = std::make_shared<JoyCtl>(); 

	// Add multithreaded executor 
	rclcpp::executors::MultiThreadedExecutor executor; 
	executor.add_node(node); 
	executor.spin(); 

	rclcpp::shutdown(); 
	return 0; 
}
