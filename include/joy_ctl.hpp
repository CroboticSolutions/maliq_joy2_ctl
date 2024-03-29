#ifndef JOY_CTL_H
#define JOY_CTL_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>

//* ros 
#include "rclcpp/rclcpp.hpp"

//* msgs
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

//* srvs
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp" 

using namespace std::chrono_literals; 
using std::placeholders::_1; 
using std::placeholders::_2; 

class JoyCtl: public rclcpp::Node 
{
	public:
		JoyCtl(); 
		//~JoyCtl(); 

	private:

		// vars
		bool 		enableJoy_; 
		mutable int scale_factor;  
	    
		// publishers	
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPub_; 
		rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr 	suctionPub_; 
		
		// subscribers
		rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joySub_; 


		// clients 
 		rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr  jingleBellsClient_; // Could be used for initing all UAVs

		void init(); 
		void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg); 

		// Setting them as const to be usable by joy_callback which is also const
		void setScaleFactor(int value); 
		int getScaleFactor() const; 
		void setEnableJoy(bool val); 
		bool getEnableJoy() const; 

		// TODO: Add service to turn joystick on and off

};

#endif


