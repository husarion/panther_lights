#!/usr/bin/env python3

import rospy

from husarion_msgs.msg import LEDAnimation, LEDAnimationArr
from husarion_msgs.srv import LEDBrightness, LEDPanel, LEDSetId, LEDSetIdRequest


class TopicPublisher:
    
    def __init__(self):
        '''topic_publisher_node class'''

        rospy.init_node('topic_publisher')

        self._battery_publisher = rospy.Publisher('/battery', Battery, self._animation_queue_callback)
        self._buttery_publisher_timer = rospy.Timer(rospy.Duration(5), self._battery_timer_callback)

        rospy.loginfo(f'{rospy.get_name()} started')

    def _battery_timer_callback(self, *args):
        battery_msg = Battery()
        battery_msg.header.stamp = rospy.Time.now()
        self._buttery_publisher_timer.publish(batery_msg)
        rospy.loginfo('publishing /battery')


def main():
    try:
        publisher = TopicPublisher()
        rospy.spin()
    except Exception as e:
        rospy.logerr(f'panther_lights error: {e}')
        exit(1)


if __name__ == '__main__':
    main()