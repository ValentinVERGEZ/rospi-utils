#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt64
from gpio_messages.msg import io_states
from gpio_messages.msg import io_description

DIPSWITCH_PIN1 = 19
DIPSWITCH_PIN2 = 15
DIPSWITCH_PIN3 = 13
DIPSWITCH_PIN4 = 11

pub = rospy.Publisher('user_identity', UInt64, queue_size=10)

def callback(data):
    dipswitch_value1=0
    dipswitch_value2=0
    dipswitch_value3=0
    dipswitch_value4=0

    for pin in data.pins:
        if pin.pinNumber == DIPSWITCH_PIN1:
            dipswitch_value1 = int(pin.pinValue)
        elif pin.pinNumber == DIPSWITCH_PIN2:
            dipswitch_value2 = int(pin.pinValue)
        elif pin.pinNumber == DIPSWITCH_PIN3:
            dipswitch_value3 = int(pin.pinValue)
        elif pin.pinNumber == DIPSWITCH_PIN4:
            dipswitch_value4 = int(pin.pinValue)

    # Dipswitch have pin 1 at left, so it will be MSB
    userid = 0
    userid += dipswitch_value1<<3
    userid += dipswitch_value2<<2
    userid += dipswitch_value3<<1
    userid += dipswitch_value4<<0

    msg = UInt64()
    msg = userid
    pub.publish(userid)

def fake_user_identifier():
    rospy.init_node('fake_user_identifier', anonymous=False)
    rospy.Subscriber("/io_states", io_states, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        fake_user_identifier()
    except rospy.ROSInterruptException:
        pass