#ifndef _GPIO_INTERFACE__FAKE_VOCAL_WAKE_NODE_
#define _GPIO_INTERFACE__FAKE_VOCAL_WAKE_NODE_

// Includes
#include <ros/ros.h>

#include "gpio_messages/io_states.h"
#include "sound_messages/WitdCommand.h"

#ifndef BUTTON_PIN
    #define BUTTON_PIN      7
#endif

// Prototypes
void io_states_sub_callback(const gpio_messages::io_statesConstPtr& msg);


#endif // _GPIO_INTERFACE__FAKE_VOCAL_WAKE_NODE_