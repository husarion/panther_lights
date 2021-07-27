#!/usr/bin/env python3

from typing import Optional

import rospy
from std_srvs.srv import SetBool

from sensor_msgs.msg import BatteryState
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from husarion_msgs.msg import LEDAnimation, LEDAnimationArr, PantherDriverStatus
from husarion_msgs.srv import LEDPanel, LEDPanelRequest, LEDAnimationId, LEDAnimationIdRequest

class LightsSchedulerNode:
    
    def __init__(self):
        '''lights_scheduler_node class'''

        self._state_animations = {
            PantherDriverStatus.STATE_ACCEPT_ALL_STATE : 'READY',
            PantherDriverStatus.STATE_DEAD_MAN_STATE : 'ERROR',
            PantherDriverStatus.STATE_JOY_STATE : 'MANUAL_ACTION',
            PantherDriverStatus.STATE_AUTONOMOUS_STATE : 'AUTONOMOUS_ACTION'
        }

        self._background_anim = {
            'READY' : (9, 'READY'),
            'ERROR' : (2, 'ERROR'),
            'MANUAL_ACTION' : (8, 'MANUAL_ACTION'),
            'AUTONOMOUS_ACTION' : (7, 'AUTONOMOUS_ACTION')
        }

        self._event_animations = {
            'E-STOP' : (1, 'E-STOP'),
            'GOAL_ARCHIVED' : (3, 'GOAL_ARCHIVED'),
            'GOAL_ABORTED' : (2, 'ERROR'),
            'CRITICAL_BATTERY' : (4, 'CRITICAL_BATTERY'),
            'LOW_BATTERY' : (5, 'LOW_BATTERY'),
            'SHOW_BATTERY' : (6, 'SHOW_BATTERY'),
        }

        self._move_base_events = {
            GoalStatus.PENDING: None,
            GoalStatus.ACTIVE: None,
            GoalStatus.PREEMPTED: (2, 'ERROR'),
            GoalStatus.SUCCEEDED: (3, 'GOAL_ARCHIVED'),
            GoalStatus.ABORTED: (2, 'ERROR'),
            GoalStatus.REJECTED: (2, 'ERROR'),
            GoalStatus.PREEMPTING: (2, 'ERROR'),
            GoalStatus.RECALLING: (2, 'ERROR'),
            GoalStatus.RECALLED: (2, 'ERROR'),
            GoalStatus.LOST: (2, 'ERROR')

        }

        self._panther_state = None
        self._last_goal_state = None
        self._battery_percentage = None
        self._last_animation_queue = None
        self._background_anim_published = False

        self._low_bat_val = rospy.get_param('low_battery_margin', 0.2)
        self._low_bat_duration = rospy.get_param('low_battery_anim_delay', 30)
        self._critical_bat_duration = rospy.get_param('critical_battery_anim_delay', 10)

        rospy.init_node('lights_scheduler_node')

        self._last_animation_queue_sub = rospy.Subscriber('/lights/controller/queue', LEDAnimationArr, self._animation_queue_callback, queue_size=10)
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

        self._clear_queue_srv()
        self._kill_current_anim_srv()

        rospy.loginfo(f'{rospy.get_name()} started')


    def _lights_scheduler(self, *args):
        '''executes rosnode'''
        pass



    def _low_battery_anim(self, *args):
        '''calls animation service with battery value'''
        self._set_animation_srv(self._event_animations['LOW_BATTERY'])


    def _critical_battery_anim(self, *args):
        '''calls animation service with battery value'''
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
        if self._critical_battery_animating:
            if key != self._event_animations['CRITICAL_BATTERY']:
                return

        rospy.wait_for_service('lights/controller/set/id')
        try:
            set_id = rospy.ServiceProxy('lights/controller/set/id', LEDAnimationId)
            req = LEDAnimationIdRequest()
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


    def _clear_queue_srv(self):
        '''clears all animations in queue'''
        rospy.wait_for_service('lights/controller/queue/clear')
        try:
            clear_queue = rospy.ServiceProxy('lights/controller/queue/clear', SetBool)
            resp = clear_queue(True)
            return resp.success
        except rospy.ServiceException as e:
            rospy.logwarn(f'service call failed: {e}')

    
    def _kill_current_anim_srv(self):
        '''removes currently running animation'''
        rospy.wait_for_service('lights/controller/queue/kill_current')
        try:
            clear_queue = rospy.ServiceProxy('lights/controller/queue/kill_current', SetBool)
            resp = clear_queue(True)
            return resp.success
        except rospy.ServiceException as e:
            rospy.logwarn(f'service call failed: {e}')

    def _remove_key_from_queue(self, key):
        '''service removing key from queue'''
        rospy.wait_for_service('lights/controller/queue/remove_id')
        try:
            remove_id = rospy.ServiceProxy('lights/controller/queue/remove_id', LEDAnimationId)
            req = LEDAnimationIdRequest()
            req.animation.id = key[0]
            req.animation.name = key[1]
            resp = remove_id(req)
            if resp.result == 'success':
                return True
        
            rospy.logwarn(f'failed to remove animation {key}')
            return False
        except rospy.ServiceException as e:
            rospy.logwarn(f'service call failed: {e}')

    
    def _animation_queue_callback(self, msg):
        '''callback subscribing to lights queue'''
        if self._last_animation_queue != msg.queue:
            self._last_animation_queue = msg.queue
            if not self._last_animation_queue:
                if self._panther_state and not self._background_anim_published:
                    key = self._background_anim[self._state_animations[self._panther_state]]
                    self._set_animation_srv(key)
                    self._background_anim_published = True
            if len(msg.queue) > 0:
                self._background_anim_published = False
        if len(msg.queue) == 2:
            anim = msg.queue[1]
            key = (anim.id, anim.name)
            if key in set(self._background_anim.values()):
                if key != self._background_anim[self._state_animations[self._panther_state]]:
                    self._remove_key_from_queue(key)




    def _battery_callback(self, msg):
        self._battery_percentage = (msg.voltage - 32) / (48 - 32)
        if self._battery_percentage <= 0.1:
            if not self._critical_battery_animating:
                if self._low_battery_anim_timer:
                    self._low_battery_animating = False
                    self._low_battery_anim_timer.shutdown()
                rospy.loginfo('starting to display critical battery message')
                self._clear_queue_srv()
                self._kill_current_anim_srv()
                self._set_animation_srv(self._event_animations['CRITICAL_BATTERY'])
                self._critical_battery_anim_timer = rospy.Timer(rospy.Duration(self._critical_bat_duration), self._critical_battery_anim)
                self._critical_battery_animating = True
                return
        elif self._battery_percentage <= self._low_bat_val:
            if not self._low_battery_animating:
                if self._critical_battery_anim_timer:
                    self._critical_battery_animating = False
                    self._critical_battery_anim_timer.shutdown()
                rospy.loginfo('starting to display low battery message')
                self._set_animation_srv(self._event_animations['LOW_BATTERY'])
                self._low_battery_anim_timer = rospy.Timer(rospy.Duration(self._low_bat_duration), self._low_battery_anim)
                self._low_battery_animating = True
                return
        else:
            if self._low_battery_animating:
                rospy.loginfo('stopping displaying low battery message')
                self._low_battery_anim_timer.shutdown()
            if self._critical_battery_animating:
                rospy.loginfo('stopping displaying critical battery message')
                self._critical_battery_anim_timer.shutdown()
            self._low_battery_animating = False
            self._critical_battery_animating = False


    def _panther_driver_status_callback(self, msg):
        self._panther_state = msg.state
        if not self._last_animation_queue:
            rospy.loginfo('setting state')
            key = self._background_anim[self._state_animations[self._panther_state]]
            self._set_animation_srv(key)
            if self._panther_state != PantherDriverStatus.STATE_AUTONOMOUS_STATE:
                self._last_goal_state = None


    def _goal_status_callback(self, msg):
        if self._panther_state == PantherDriverStatus.STATE_AUTONOMOUS_STATE:
            if msg.status_list:
                if self._last_goal_state is None or self._last_goal_state != msg.status_list[0].status:
                    self._last_goal_state = msg.status_list[0].status
                    anim = self._move_base_events[msg.status_list[0].status]
                    if anim:
                        self._set_animation_srv(anim)

def main():
    try:
        lights = LightsSchedulerNode()
        rospy.spin()
    except Exception as e:
        rospy.logerr(f'panther_lights_scheduler error: {e}')
        exit(1)


if __name__ == '__main__':
    main()