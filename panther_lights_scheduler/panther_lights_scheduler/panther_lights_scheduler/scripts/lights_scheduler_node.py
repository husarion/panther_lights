#!/usr/bin/env python3

import rospy
from panther_lights.srv import Brightness, PanelState, Animation, ImageAnimation


class LightsSchedulerNode:
    
    def __init__(self,
                 config_path=None):
        '''lights_scheduler_node class'''

        self._current_animation = None

        rospy.init_node('lights_scheduler_node')
        self._set_lights_service = rospy.Service('set/id', Animation, self._set_lights)
        self._set_image_animation_service = rospy.Service('set/image', ImageAnimation, self._set_image_animation)
        self._set_brightness_service = rospy.Service('brightness', Brightness, self._set_brightness)
        self._set_panel_state_service = rospy.Service('panel_state', PanelState, self._set_panel_state)
        self._lights_scheduler_timer = rospy.Timer(rospy.Duration(0.05), self._lights_scheduler)
        self._battery_anim_timer = rospy.Timer(rospy.Duration(10), self._battery_anim_timer)

        rospy.loginfo('panther_lights started')


    def _lights_scheduler(self, *args):
        '''executes rosnode'''
        pass


    def _battery_anim_timer(self, *args):
        '''calls animation service with battery vallue'''
        rospy.loginfo('setting battery animation')


    def _set_animation_srv(self, key, param):
        '''calls animation service'''
        pass


    def _battery_callback(self, msg):
        pass


    def _panther_driver_callback(self, msg):
        pass


def main():
    try:
        lights = LightsSchedulerNode()
        rospy.spin()
    except Exception as e:
        rospy.logerr(f'panther_lights error: {e}')
        exit(1)


if __name__ == '__main__':
    main()