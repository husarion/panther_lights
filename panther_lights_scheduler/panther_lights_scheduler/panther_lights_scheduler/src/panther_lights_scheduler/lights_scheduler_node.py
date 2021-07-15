#!/usr/bin/env python3

from typing import Optional

import rospy

from husarion_msgs.msg import LEDAnimation, LEDAnimationArr
from husarion_msgs.srv import LEDBrightness, LEDPanel, LEDPanelRequest, LEDSetId, LEDSetIdRequest


class LightsSchedulerNode:
    
    def __init__(self):
        '''lights_scheduler_node class'''

        self._current_state = None
        self._aniamtion_queue = None

        self._state_keys = {
            1 : (1, 'E-STOP')
        }
        self._battery_percentage = 0.5
        

        rospy.init_node('lights_scheduler_node')

        self._animation_queue_sub = rospy.Subscriber('/queue', LEDAnimationArr, self._animation_queue_callback)
        self._lights_scheduler_timer = rospy.Timer(rospy.Duration(5), self._lights_scheduler)
        self._battery_anim_timer = rospy.Timer(rospy.Duration(10), self._battery_anim_timer)

        # Enable LED panels
        self._set_panel_srv(0, True)
        self._set_panel_srv(1, True)

        rospy.loginfo(f'{rospy.get_name()} started')


    def _lights_scheduler(self, *args):
        '''executes rosnode'''
        self._set_animation_srv(self._state_keys[1])
        rospy.loginfo('setting error anim')


    def _battery_anim_timer(self, *args):
        '''calls animation service with battery vallue'''
        rospy.loginfo('setting battery animation')
        self._set_animation_srv((6, 'SHOW_BATTERY'), str(self._battery_percentage))

    
    def _set_panel_srv(self, panel, state):
        '''sets LED panel state'''
        rospy.wait_for_service('panel_state')
        try:
            panel_state = rospy.ServiceProxy('panel_state', LEDPanel)
            req = LEDPanelRequest()
            req.panel = panel
            req.data = state
            resp = panel_state(req)
            if resp.result == 'success':
                return True
        
            rospy.logwarn(f'failed to set panel {panel} to state {state}')
            return False
        except rospy.ServiceException as e:
            rospy.logwarn(f'service call failed: {e}')


    def _set_animation_srv(self, key, param: Optional[str]='0'):
        '''calls animation service'''
        rospy.wait_for_service('set/id')
        try:
            set_id = rospy.ServiceProxy('set/id', LEDSetId)
            req = LEDSetIdRequest()
            req.animation.id = key[0]
            req.animation.name = key[1]
            req.param = param
            resp = set_id(req)
            if resp.result == 'success':
                return True
        
            rospy.logwarn(f'failed to set animation {key}')
            return False
        except rospy.ServiceException as e:
            rospy.logwarn(f'service call failed: {e}')

    
    def _animation_queue_callback(self, msg):
        '''callback subscrybing to lights queue'''
        self._aniamtion_queue = msg.queue
        if not self._aniamtion_queue:
            rospy.loginfo('got queue')
            # key = self._state_keys[self._panther_state.state]
            # self._set_animation_srv(key)


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