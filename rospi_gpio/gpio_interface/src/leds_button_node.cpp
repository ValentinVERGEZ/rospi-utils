#include <gpio_interface/leds_button_node.h>

//-- Global variable
unsigned int RGB_led_mode		= RED_GRADUAL_BLINK;
unsigned int right_led_state	= 0;
unsigned int left_led_state		= 0;

//-- Main
int main(int argc, char** argv)
{
	ros::init(argc, argv, "leds_button_node");
	ros::NodeHandle nh;
	ros::Rate loop_rate(5);
    ros::Publisher state_pub = nh.advertise<gpio_messages::io_states>("io_states", 1000, true);
    ros::Subscriber command_sub = nh.subscribe("io_commands", 1, command_sub_callback);
    ros::Subscriber cloud_state_sub = nh.subscribe("witd_output", 1, cloud_state_sub_callback);

	//-- Init wiringPi
    init_wiring_pi();

	while(nh.ok()) {
		ros::spinOnce();

		// Recover button and dipswitch states
		int button_value = digitalRead(BUTTON_PIN);
		int dipswitch_value1 = digitalRead(DIPSWITCH_PIN1);
		int dipswitch_value2 = digitalRead(DIPSWITCH_PIN2);
		int dipswitch_value3 = digitalRead(DIPSWITCH_PIN3);
		int dipswitch_value4 = digitalRead(DIPSWITCH_PIN4);
		// Publish state
		gpio_messages::io_states msg;
		gpio_messages::io_description io;
		io.pinNumber = BUTTON_PIN;
		io.pinMode = io.DIGITAL_IN;
		io.pinValue = button_value;
		msg.stamp = ros::Time::now();
		msg.pins.push_back(io);

		io.pinNumber = DIPSWITCH_PIN1;
		io.pinMode = io.DIGITAL_IN;
		io.pinValue = dipswitch_value1;
		io.pinMode = io.DIGITAL_IN;
		msg.pins.push_back(io);

		io.pinNumber = DIPSWITCH_PIN2;
		io.pinMode = io.DIGITAL_IN;
		io.pinValue = dipswitch_value2;
		io.pinMode = io.DIGITAL_IN;
		msg.pins.push_back(io);

		io.pinNumber = DIPSWITCH_PIN3;
		io.pinMode = io.DIGITAL_IN;
		io.pinValue = dipswitch_value3;
		io.pinMode = io.DIGITAL_IN;
		msg.pins.push_back(io);

		io.pinNumber = DIPSWITCH_PIN4;
		io.pinMode = io.DIGITAL_IN;
		io.pinValue = dipswitch_value4;
		io.pinMode = io.DIGITAL_IN;
		msg.pins.push_back(io);

		state_pub.publish(msg);

		// Write on Right and Left RED LEDs
		digitalWrite(RIGHT_LED_PIN, right_led_state);
		digitalWrite(LEFT_LED_PIN, left_led_state);

		// Write on RGB LED
		RGB_led_write(RGB_led_mode);

		loop_rate.sleep();
	}

	return 0;
}

//-- Callbacks
void command_sub_callback(const gpio_messages::io_statesConstPtr& msg)
{
	for(int index=0; index < msg->pins.size(); index++)
	{
		// Manage only known outputs
		switch(msg->pins[index].pinNumber)
		{
			case LEFT_LED_PIN:
			{
				left_led_state = msg->pins[index].pinValue;

				//-- Debug purpose
				static unsigned int prec_left_led_state = left_led_state+1;
				if(prec_left_led_state != left_led_state) {
					prec_left_led_state = left_led_state;

					ROS_INFO_STREAM("LEFT_LED, state : " << left_led_state);
				}
			} break;

			case RIGHT_LED_PIN:
			{
				right_led_state = msg->pins[index].pinValue;

				//-- Debug purpose
				static unsigned int prec_right_led_state = right_led_state+1;
				if(prec_right_led_state != right_led_state) {
					prec_right_led_state = right_led_state;

					ROS_INFO_STREAM("RIGHT_LED, state : " << right_led_state);
				}
			} break;
		}
	}
}

void cloud_state_sub_callback(const sound_messages::WitdOutputConstPtr& msg)
{
	switch(msg->state) 
	{
		case sound_messages::WitdOutput::IDLE:
			RGB_led_mode = BLUE_GRADUAL_BLINK;
		break;

		case sound_messages::WitdOutput::TRANSFERRING_DATA:
			RGB_led_mode = YELLOW;
		break;

		case sound_messages::WitdOutput::RECORDING:
			RGB_led_mode = RED;
		break;

		default:
			RGB_led_mode = RED_GRADUAL_BLINK;
			break;
	}
}

//-- WiringPi utils
void init_wiring_pi()
{	
	//-- Init wiringPi
	// Setup wiringPi with a physical pin number referencing
	wiringPiSetupPhys();
	// Configure standards LED pins
	pinMode(RIGHT_LED_PIN, OUTPUT);
	pinMode(LEFT_LED_PIN, OUTPUT);
	// Configure PUSH BUTTON pin
	pinMode(BUTTON_PIN, INPUT);
	pullUpDnControl(BUTTON_PIN, PUD_UP);	
	// Configure DIPSWITCH pins
	pinMode(DIPSWITCH_PIN1, INPUT);
	pullUpDnControl(DIPSWITCH_PIN1, PUD_UP);	
	pinMode(DIPSWITCH_PIN2, INPUT);
	pullUpDnControl(DIPSWITCH_PIN2, PUD_UP);
	pinMode(DIPSWITCH_PIN3, INPUT);
	pullUpDnControl(DIPSWITCH_PIN3, PUD_UP);
	pinMode(DIPSWITCH_PIN4, INPUT);
	pullUpDnControl(DIPSWITCH_PIN4, PUD_UP);
	// Configure RGB LED pins
	pinMode(RGB_RED_PIN, OUTPUT);
	pinMode(RGB_GREEN_PIN, OUTPUT);
	pinMode(RGB_BLUE_PIN, PWM_OUTPUT);
}

void RGB_led_write(unsigned int mode)
{
	//-- Debug purpose
	static unsigned int precMode = mode+1;
	if(precMode != mode) {
		precMode = mode;

		std::string mode_str;
		switch (mode) {
			case BLUE_GRADUAL_BLINK:
				mode_str = "BLUE_GRADUAL_BLINK"; break;
			case RED_GRADUAL_BLINK:
				mode_str = "RED_GRADUAL_BLINK"; break;
			case RED:
				mode_str = "RED"; break;
			case YELLOW:
				mode_str = "YELLOW"; break;
			default:
				mode_str = "unknown : ",
				mode_str += mode;
				break;
		}

		ROS_INFO_STREAM("RGB_led_write, mode : " << mode_str);
	}

	// Recover the 0-255 red, green, and blue values
	// and convert them to 0-1024 values (255*4 = 1020)  
	unsigned int red 	= ((mode & 0xFF0000) >> 16)	* 4;
	unsigned int green 	= ((mode & 0x00FF00) >> 8)	* 4;
	unsigned int blue 	= ((mode & 0x0000FF) >> 0)	* 4;

	// Recover the behavior code (normal, blnking, etc)
	unsigned int behavior	= (mode & 0xFF000000) >> 24;

	// Ramp for gradual blink
	#define MAX_BLINK_RAMP		2000
	#define MIDLE_BLINK_RAMP	(MAX_BLINK_RAMP/2)
	static unsigned int blink_ramp = 0;
	blink_ramp+=30;
	if(blink_ramp > MAX_BLINK_RAMP) { blink_ramp=0; }

	switch(behavior)
	{
		// Gradual blink behavior
		case 0x01:
		{
		// RED LED
			unsigned int red_new_value=0;
			if(blink_ramp<MIDLE_BLINK_RAMP) { red_new_value = blink_ramp*red/MIDLE_BLINK_RAMP; }
			else { red_new_value = (blink_ramp-MIDLE_BLINK_RAMP)*red/MIDLE_BLINK_RAMP; }

			if(RED_IS_PWM) {
				pwmWrite(RGB_RED_PIN, red_new_value); 
			}
			else {
				// Switch ON for red > 0.5*MAX (MAX=1024)
				digitalWrite(RGB_RED_PIN, red_new_value/515);
			}

		// GREEN LED
			unsigned int green_new_value=0;
			if(blink_ramp<MIDLE_BLINK_RAMP) { green_new_value = blink_ramp*green/MIDLE_BLINK_RAMP; }
			else { green_new_value = (blink_ramp-MIDLE_BLINK_RAMP)*green/MIDLE_BLINK_RAMP; }

			if(GREEN_IS_PWM) {
				pwmWrite(RGB_GREEN_PIN, green_new_value);
			}
			else {
				// Switch ON for green > 0.5*MAX (MAX=1024)
				digitalWrite(RGB_GREEN_PIN, green_new_value/515);
			}

		// BLUE LED
			unsigned int blue_new_value=0;
			if(blink_ramp<MIDLE_BLINK_RAMP) { blue_new_value = blink_ramp*blue/MIDLE_BLINK_RAMP; }
			else { blue_new_value = blue - (blink_ramp-MIDLE_BLINK_RAMP)*blue/MIDLE_BLINK_RAMP; }

			if(BLUE_IS_PWM) {
				pwmWrite(RGB_BLUE_PIN, blue_new_value);
			}
			else {
				// Switch ON for blue > 0.5*MAX (MAX=1024)
				digitalWrite(RGB_BLUE_PIN, blue_new_value/515);
			}
		}
		break;

		// Normal and unknown behavior
		case 0x00:
		default:
		// RED LED
			if(RED_IS_PWM) {
				pwmWrite(RGB_RED_PIN, red);
			}
			else {
				// Switch ON for red > 0.5*MAX (MAX=1024)
				digitalWrite(RGB_RED_PIN, red/515);
			}

		// GREEN LED
			if(GREEN_IS_PWM) {
				pwmWrite(RGB_GREEN_PIN, green);
			}
			else {
				// Switch ON for green > 0.5*MAX (MAX=1024)
				digitalWrite(RGB_GREEN_PIN, green/515);
			}

		// BLUE LED
			if(BLUE_IS_PWM) {
				pwmWrite(RGB_BLUE_PIN, blue);
			}
			else {
				// Switch ON for blue > 0.5*MAX (MAX=1024)
				digitalWrite(RGB_BLUE_PIN, blue/515);
			}
		break;
	}
}
