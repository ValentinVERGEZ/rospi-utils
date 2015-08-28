#ifndef _GPIO_INTERFACE__FAKE_VOCAL_INTERPRETER_NODE_
#define _GPIO_INTERFACE__FAKE_VOCAL_INTERPRETER_NODE_

// Includes
#include <ros/ros.h>

#include "gpio_messages/io_states.h"
#include "sound_messages/WitdOutput.h"

// Defines
#define RIGHT_LED_PIN   3
#define LEFT_LED_PIN    5

// Prototypes
void witd_output_sub_callback(const sound_messages::WitdOutputConstPtr& msg);


#endif // _GPIO_INTERFACE__FAKE_VOCAL_INTERPRETER_NODE_