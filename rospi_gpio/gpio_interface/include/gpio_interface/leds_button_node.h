#ifndef _GPIO_INTERFACE__LEDS_BUTTON_NODE_
#define _GPIO_INTERFACE__LEDS_BUTTON_NODE_

//-- Includes
#include <ros/ros.h>
#include <algorithm>
#include <string>

#include "gpio_messages/io_states.h"
#include "sound_messages/WitdOutput.h"

// Be carreful, this file une devil #define, it have to be included in last
#include "wiringPi.h"

//-- Defines
// Pin numbers
#define PWM_PIN         12

#define RGB_RED_PIN     8
#define RED_IS_PWM      false
#define RGB_GREEN_PIN   10
#define GREEN_IS_PWM    false
#define RGB_BLUE_PIN    12
#define BLUE_IS_PWM     true

#define BUTTON_PIN      7
#define RIGHT_LED_PIN   3
#define LEFT_LED_PIN    5
#define DIPSWITCH_PIN1  19
#define DIPSWITCH_PIN2  15
#define DIPSWITCH_PIN3  13
#define DIPSWITCH_PIN4  11

// RGB LED modes (RED, YELLOW, BLUE_GRADUAL_BLINK)
//      The two upper digits are for the mode
//      00 mean normal mode
//      01 mean for a gradual blink
//      
//      The three other are a normal HEX RGB color code
#define RED                 0x00FF0000
#define YELLOW              0x00FFFF00
#define BLUE_GRADUAL_BLINK  0x010000FF
#define RED_GRADUAL_BLINK   0x01FF0000

//-- Macros (used to silence errors when wiringPi doesn't exist)
// #define WIRING_PI_NOT_INSTALLED
#ifdef WIRING_PI_NOT_INSTALLED
    #define pwmWrite(x, y);
    #define digitalWrite(x, y);
    #define digitalRead(x) 0
    #define wiringPiSetupPhys();
    #define pinMode(x, y);
    #define pullUpDnControl(x, y);
#endif

//-- Prototypes
void command_sub_callback(const gpio_messages::io_statesConstPtr& msg);
void cloud_state_sub_callback(const sound_messages::WitdOutputConstPtr& msg);
void init_wiring_pi();
void RGB_led_write(unsigned int mode);


#endif // _GPIO_INTERFACE__LEDS_BUTTON_NODE_
