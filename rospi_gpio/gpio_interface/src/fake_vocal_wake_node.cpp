#include <gpio_interface/fake_vocal_wake_node.h>

//-- Global variables
unsigned long int actualRequestId=0;
unsigned short int requestType=-1;
bool reqAvailable = false;

//-- Main
int main(int argc, char** argv)
{
    ros::init(argc, argv, "fake_vocal_wake_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    ros::ServiceClient command_client= nh.serviceClient<sound_messages::WitdCommand>("witd_command");
    ros::Subscriber io_states_sub = nh.subscribe("io_states", 1, io_states_sub_callback);

    while(nh.ok()) {
        ros::spinOnce();

        if(reqAvailable) {
            sound_messages::WitdCommand srv;
            srv.request.requestId=actualRequestId;
            srv.request.requestType=requestType;
            command_client.call(srv);
            ROS_INFO("Req accepted : %s\n%s",
                ((srv.response.accepted)?"true":"false"),
                srv.response.message.c_str());
            reqAvailable = false;
        }

        loop_rate.sleep();
    }

    return 0;
}

void io_states_sub_callback(const gpio_messages::io_statesConstPtr& msg)
{
    bool fallingEdge = false;
    static float precButtonValue = -1;
    for(int index=0; index < msg->pins.size(); index++)
    {
        // Only manage the button event
        if(msg->pins[index].pinNumber==BUTTON_PIN) {
            if(precButtonValue==msg->pins[index].HIGH && msg->pins[index].pinValue==msg->pins[index].LOW)
                fallingEdge=true;
            precButtonValue=msg->pins[index].pinValue;
            if(fallingEdge) {
                switch(requestType)
                {
                    case sound_messages::WitdCommandRequest::START:
                    {
                        requestType=sound_messages::WitdCommandRequest::STOP;
                        reqAvailable=true;
                    }
                        break;

                    case sound_messages::WitdCommandRequest::STOP:
                    default:
                    {
                        requestType=sound_messages::WitdCommandRequest::START;
                        actualRequestId++;
                        reqAvailable=true;
                    }
                        break;
                }
            }
        }
    }
}
