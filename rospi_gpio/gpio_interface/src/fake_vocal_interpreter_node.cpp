vveinclude <gpio_interface/fake_vocal_interpreter_node.h>
// #include <wiringPi.h>
 
gpio_messages::io_states _command_msg;

// Main
int main(int argc, char** argv)
{
    ros::init(argc, argv, "fake_vocal_interpreter_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    ros::Subscriber witd_output_sub = nh.subscribe("witd_output", 1, witd_output_sub_callback);
    ros::Publisher commands_pub = nh.advertise<gpio_messages::io_states>("io_commands", 1000, true);

    while(nh.ok()) {
        ros::spinOnce();

        _command_msg.stamp = ros::Time::now();
        commands_pub.publish(_command_msg);

        loop_rate.sleep();
    }

    return 0;
}

void witd_output_sub_callback(const sound_messages::WitdOutputConstPtr& msg)
{
    static unsigned int lastRequestId = 999;

    if(msg->actual_request_id != lastRequestId && msg->state == sound_messages::WitdOutput::IDLE) {
        lastRequestId = msg->actual_request_id;

        gpio_messages::io_description pinDescRight;
        gpio_messages::io_description pinDescLeft;
        if(msg->actual_request_id %2 ==0) {
            pinDescRight.pinNumber = RIGHT_LED_PIN;
            pinDescRight.pinMode = pinDescRight.DIGITAL_OUT;
            pinDescRight.pinValue = pinDescRight.HIGH;

            pinDescLeft.pinNumber = LEFT_LED_PIN;
            pinDescLeft.pinMode = pinDescLeft.DIGITAL_OUT;
            pinDescLeft.pinValue = pinDescLeft.LOW;           
        }
        else {
            pinDescRight.pinNumber = RIGHT_LED_PIN;
            pinDescRight.pinMode = pinDescRight.DIGITAL_OUT;
            pinDescRight.pinValue = pinDescRight.LOW;

            pinDescLeft.pinNumber = LEFT_LED_PIN;
            pinDescLeft.pinMode = pinDescLeft.DIGITAL_OUT;
            pinDescLeft.pinValue = pinDescLeft.HIGH;    
        }
        _command_msg.pins.clear();
        _command_msg.pins.push_back(pinDescRight);
        _command_msg.pins.push_back(pinDescLeft);

        ROS_INFO("New request : %d", msg->actual_request_id);
    }
}