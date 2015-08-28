#!/usr/bin/env python
import rospy
from sound_messages.msg import WitdOutput
from gpio_messages.msg import io_states
from gpio_messages.msg import io_description
import json

RIGHT_LED_PIN = 3
LEFT_LED_PIN =  5
pub = rospy.Publisher('io_commands', io_states, queue_size=10)

def callback(data):
    try:
        jsonObject = json.loads(data.raw_data)
        if jsonObject["outcomes"][0]["intent"]=="leds" :
            onOff = jsonObject["outcomes"][0]["entities"]["on_off"][0]["value"]
            whichLed = jsonObject["outcomes"][0]["entities"]["led_choice"][0]["value"]
#            rospy.loginfo("---------\n"+whichLed + " : " + onOff+"\n")
    except Exception, e:
        return

    message = io_states()
    description = io_description()
    message.stamp = rospy.get_rostime()


    if whichLed == "right" :
        description.pinNumber = RIGHT_LED_PIN
    elif whichLed == "left" :
        description.pinNumber = LEFT_LED_PIN
    else :
        return
    
    if onOff == "on" :
        description.pinValue = description.HIGH
    elif onOff == "off" :
        description.pinValue = description.LOW
    else :
        return

    description.pinMode = description.DIGITAL_OUT

    message.pins.append(description)

    pub.publish(message);

def simple_json_vocal_interpreter():
    rospy.init_node('simple_json_vocal_interpreter', anonymous=False)
    rospy.Subscriber("/witd_output", WitdOutput, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        simple_json_vocal_interpreter()
    except rospy.ROSInterruptException:
        pass
