#!/usr/bin/env python3

from typing import Optional

import rospy

from sensor_msgs.msg import BatteryState
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from husarion_msgs.msg import LEDAnimation, LEDAnimationArr, PantherDriverStatus
from husarion_msgs.srv import LEDPanel, LEDPanelRequest, LEDSetId, LEDSetIdRequest

class LightsSchedulerNode:
    
    def __init__(self):
        '''lights_scheduler_node class'''

        self._current_state = None
        self._aniamtion_queue = None

        self._state_animations = {
            PantherDriverStatus.STATE_ACCEPT_ALL_STATE : 'READY',
            PantherDriverStatus.STATE_DEAD_MAN_STATE : 'ERROR',
            PantherDriverStatus.STATE_JOY_STATE : 'MANUAL_ACTION',
            PantherDriverStatus.STATE_AUTONOMUS_STATE : 'AUTONOMUS_ACTION'
        }

        self._backgorund_anim = {
            'READY' : (9, 'READY'),
            'ERROR' : (2, 'ERROR'),
            'MANUAL_ACTION' : (8, 'MANUAL_ACTION'),
            'AUTONOMUS_ACTION' : (7, 'AUTONOMUS_ACTION')
        }

        self._event_animations = {
            'E-STOP' : (1, 'E-STOP'),
            'GOAL_ARCHIVED' : (3, 'GOAL_ARCHIVED'),
            'GOAL_ABORTED' : (2, 'ERROR'),
            'CRITICAL_BATTERY' : (4, 'CRITICAL_BATTERY'),
            'LOW_BATTERY' : (5, 'LOW_BATTERY'),
            'SHOW_BATTERY' : (6, 'SHOW_BATTERY'),
        }

        self._battery_percentage = None
        self._panther_state = None
        self._current_goal = None
        self._last_anim = None

        rospy.init_node('lights_scheduler_node')

        self._animation_queue_sub = rospy.Subscriber('/lights/controller/queue', LEDAnimationArr, self._animation_queue_callback, queue_size=10)
        self._battery_sub = rospy.Subscriber('/battery', BatteryState, self._battery_callback)
        self._goal_status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self._goal_status_callback)
        self._driver_manager_state_sub = rospy.Subscriber('/panther_driver/manager/status', PantherDriverStatus, self._panther_driver_status_callback)
        self._lights_scheduler_timer = rospy.Timer(rospy.Duration(5), self._lights_scheduler)

        self._low_battery_anim_timer = None
        self._critical_battery_anim_timer = None

        self._low_battery_animating = False
        self._critical_battery_animating = False

        # Enable LED panels
        self._set_panel_srv(0, True)
        self._set_panel_srv(1, True)

        rospy.loginfo(f'{rospy.get_name()} started')


    def _lights_scheduler(self, *args):
        '''executes rosnode'''
        pass



    def _low_battery_anim(self, *args):
        '''calls animation service with battery vallue'''
        self._set_animation_srv(self._event_animations['LOW_BATTERY'])


    def _critical_battery_anim(self, *args):
        '''calls animation service with battery vallue'''
        self._set_animation_srv(self._event_animations['CRITICAL_BATTERY'])

    
    def _set_panel_srv(self, panel, state):
        '''sets LED panel state'''
        rospy.wait_for_service('lights/controller/panel_state')
        try:
            panel_state = rospy.ServiceProxy('lights/controller/panel_state', LEDPanel)
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


    def _set_animation_srv(self, key, param: Optional[str]=''):
        '''calls animation service'''
        rospy.logwarn(f'setting animation {key}, with param: \'{param}\'')
        rospy.wait_for_service('lights/controller/set/id')
        try:
            set_id = rospy.ServiceProxy('lights/controller/set/id', LEDSetId)
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


    def _clear_animation_from_queue_srv(self, key):
        '''calls animation service'''
        rospy.logwarn(f'setting animation {key}, with param: \'{param}\'')
        rospy.wait_for_service('lights/controller/clear/animation_from_queue')
        try:
            set_id = rospy.ServiceProxy('lights/controller/clear/animation_from_queue', LEDSetId)
            req = LEDSetIdRequest()
            req.animation.id = key[0]
            req.animation.name = key[1]
            req.param = ''
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
            if self._last_anim and not self._last_anim[0] in set([anim[0] for anim in self._backgorund_anim.values()]):
                if self._panther_state:
                    key = self._backgorund_anim[self._state_animations[self._panther_state]]
                    self._set_animation_srv(key)
        
        elif len(self._aniamtion_queue) == 1:
            self._last_anim = (self._aniamtion_queue[0].id, self._aniamtion_queue[0].name)


    def _battery_callback(self, msg):
        self._battery_percentage = (msg.voltage - 32) / (48 - 32)
        if self._battery_percentage <= 0.1 and not self._critical_battery_animating:
            if self._low_battery_anim_timer:
                self._low_battery_anim_timer.shutdown()
            rospy.loginfo('starting to display critical battery message')
            self._critical_battery_anim_timer = rospy.Timer(rospy.Duration(10), self._critical_battery_anim)
            self._critical_battery_animating = True
            return
        elif self._battery_percentage <= 0.2 and not self._low_battery_animating:
            if self._critical_battery_anim_timer:
                self._critical_battery_anim_timer.shutdown()
            rospy.loginfo('starting to display low battery message')
            self._low_battery_anim_timer = rospy.Timer(rospy.Duration(30), self._low_battery_anim)
            self._low_battery_animating = True
            return
        else:
            if self._low_battery_anim_timer:
                rospy.loginfo('stopping displaying low battery message')
                self._low_battery_anim_timer.shutdown()
            if self._critical_battery_anim_timer:
                rospy.loginfo('stopping displaying critical battery message')
                self._critical_battery_anim_timer.shutdown()
            self._low_battery_animating = False
            self._critical_battery_animating = False


    def _panther_driver_status_callback(self, msg):
        self._panther_state = msg.state
        if not self._aniamtion_queue:
            rospy.loginfo('setting state')
            key = self._backgorund_anim[self._state_animations[self._panther_state]]
            self._set_animation_srv(key)


    def _goal_status_callback(self, msg):
        if self._panther_state == PantherDriverStatus.STATE_AUTONOMUS_STATE:
            last_goal = self._current_goal
            for goal in msg.status_list:
                if goal.status == GoalStatus.ACTIVE:
                    self._current_goal = goal
                    break
            if last_goal.goal_id.id == self._current_goal.goal_id.id:
                if goal.status == GoalStatus.SUCCEEDED:
                    self._set_animation_srv(self._event_animations['GOAL_ARCHIVED'])
                    return

            if goal.status != GoalStatus.PENDING:
                self._set_animation_srv(self._event_animations['GOAL_ABORTED'])


def main():
    try:
        lights = LightsSchedulerNode()
        rospy.spin()
    except Exception as e:
        rospy.logerr(f'panther_lights error: {e}')
        exit(1)


if __name__ == '__main__':
    main()