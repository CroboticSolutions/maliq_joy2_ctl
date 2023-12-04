#include "joy_ctl.hpp"

JoyCtl::JoyCtl(): Node("joy_ctl")
{
    init();

    setScaleFactor(1); 

    enableJoy_ = true; 

}

void JoyCtl::init()
{

    // publishers
    cmdVelPub_ 		    = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1); 

    // subscribers
    joySub_ 		    = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&JoyCtl::joy_callback, this, _1)); 

    RCLCPP_INFO(this->get_logger(), "Initialized joy_ctl"); 
}

void JoyCtl::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) 
{   
	float x_dir, y_dir, yaw;  
	std::vector<float> axes_ = msg->axes; 
	
        x_dir = axes_.at(3); 
	y_dir = axes_.at(2); 
	yaw = axes_.at(4);

    // Enabling joystick functionality
    // R2 pressed --> joy on
    if (msg->buttons.at(7) == 1)
    { 
        RCLCPP_INFO(this->get_logger(), "Turning joystick on!");    
        setEnableJoy(true); 
    }

    // R2 released --> joy off
    if (msg->buttons.at(7) == 0)
    {
        
        RCLCPP_INFO(this->get_logger(), "Turning joystick off!"); 
        setEnableJoy(false); 
    }

    enableJoy_ = getEnableJoy(); 

    float sF_; 
    int sF = getScaleFactor();
    // https://www.quantstart.com/articles/Passing-By-Reference-To-Const-in-C/ 
    if (msg->buttons.at(5) == 1){
        // crazy flight mode
        sF_ = sF; 
        RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "[OPERATION_MODE]: Crazy!"); 

    }else{
        // normal flight mode
        sF_ = static_cast<float>(sF) / 10.0;  
        RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "[OPERATION MODE]: Slow!"); 
    }
	
	//hardcode scaling factor for test
	sF_ = 50; 
	// Create teleop msg
	auto teleop_msg 	    = geometry_msgs::msg::Twist(); 
	teleop_msg.linear.x	    = x_dir  * sF_; 
	teleop_msg.linear.y 	= y_dir   * sF_; 
	teleop_msg.angular.z 	= yaw    * sF_; 
    
    if (enableJoy_){
        cmdVelPub_->publish(teleop_msg); 
    }
    else{
        teleop_msg.linear.x = 0;
        teleop_msg.linear.y = 0;
        teleop_msg.angular.z = 0;
	cmdVelPub_->publish(teleop_msg); 
    }

    // △ --> increase scale factor by one
    if (msg->buttons.at(3) == 1){
        sF++; 
        setScaleFactor(sF);  
        RCLCPP_INFO_STREAM(this->get_logger(), "Increasing scale factor: " << scale_factor);
    }
    
    // O --> reset scale factor on one
    if (msg->buttons.at(1) == 1){
        RCLCPP_INFO_STREAM(this->get_logger(), "Resetting scale factor."); 
        setScaleFactor(1); 
    }

}

// Methods that set scale factor 
void JoyCtl::setScaleFactor(int value)
{
    scale_factor = value; 
}

int JoyCtl::getScaleFactor() const
{
    return scale_factor; 
}

void JoyCtl::setEnableJoy(bool val) 
{
    enableJoy_ = val; 
}

bool JoyCtl::getEnableJoy() const
{
    return enableJoy_; 
}
