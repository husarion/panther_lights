#!/usr/bin/env python3

import os
import time
import yaml
import copy
import queue
import signal

import rospy
from std_srvs.srv import Empty

from panther_lights.event import Event
from panther_lights.controller import Controller
from panther_lights.led_config_importer import LEDConfigImporter
from panther_lights.msg import Animation, ImageAnimation
from panther_lights.srv import Brightness, PanelState

class LightsNode:
    
    def __init__(self,
                 config_path=None):
        '''lights_node class'''
        if config_path is None:
            config_path = '../config/led_conf.yaml'

        src_path = os.path.dirname(os.path.abspath(__file__))
        conf_path = os.path.join(src_path, config_path)
        conf_file = open(conf_path,'r')
        conf_yaml = yaml.load(conf_file, Loader=yaml.Loader)
        conf_file.close()

        self._led_config_importer = LEDConfigImporter(conf_yaml)
        self._num_led = self._led_config_importer.num_led
        self._global_brightness = self._led_config_importer.global_brightness

        self._controller = Controller(num_led=self._num_led,
                                      brightness=self._global_brightness)

        self._anim_queue = []
        self._interrupt = False

        rospy.init_node('lights_node')
        self._rate = rospy.Rate(10)
        self._set_lights_sub = rospy.Subscriber('set_lights', Animation, self._set_lights)
        self._set_image_animation_sub = rospy.Subscriber('set_image_animation', ImageAnimation, self._set_image_animation)
        self._set_brightness_service = rospy.Service('set_brightness', Brightness, self._set_brightness)
        self._set_panel_state_service = rospy.Service('set_panel_state', PanelState, self._set_panel_state)


    def run(self):
        '''executes rosnode'''
        rospy.loginfo('panther_lights started')
        current_animation = None
        while not rospy.is_shutdown():
            if current_animation:
                if current_animation.finished:
                    del current_animation
                    current_animation = None

                # animation interrupts and saves previous
                if self._anim_queue and self._interrupt:
                    self._interrupt = False
                    current_animation.stop()
                    if current_animation.percent_done > 0.9:
                        del current_animation
                    else:
                        if current_animation.percent_done < 0.15:
                            current_animation.reset()
                        self._anim_queue.insert(1, current_animation)

                    current_animation = self._anim_queue.pop(0)
                    if not current_animation.spawned:
                        current_animation.spawn(self._controller)
                    current_animation.run()
            else:
                # await new animations
                while not self._anim_queue and not rospy.is_shutdown():
                    self._rate.sleep()
                if rospy.is_shutdown():
                    break

                # get, spawn and start new animation
                current_animation = self._anim_queue.pop(0)
                if not current_animation.spawned:
                    current_animation.spawn(self._controller)
                current_animation.run()
            self._rate.sleep()
        

    def _set_lights(self, anim):
        '''rosservice callback for /set_lights'''
        try:
            if anim.id != 0 and not anim.name:
                animation = self._led_config_importer.get_event(id=anim.id)
            elif anim.id == 0 and anim.name:
                animation = self._led_config_importer.get_event(name=anim.name)
            else:
                animation = self._led_config_importer.get_event(id=anim.id, name=anim.name)
        except Exception as e:
            return f'failed'

        animation.param = anim.param
        if animation.interrupting:
            self._anim_queue.insert(0, animation)
            self._interrupt = True
        else:
            self._anim_queue.append(animation)

        return f'success'


    def _set_image_animation(self, image):
        '''rosservice callback for /set_image_animation'''
        if not image.image_front or not image.image_tail or image.duration == 0:
            return 'failed'

        animation = {
            'repeat':     1,
            'duration':   0,
            'keep_state': False,
            'image':      0
        }
        event = {
            'front': copy.copy(animation),
            'tail':  copy.copy(animation)
        }

        try:
            if image.color:
                event['front']['color'] = int(image.color, 16)
                event['tail']['color'] = int(image.color, 16)

            if image.repeat != 0:
                event['front']['repeat'] = image.repeat
                event['tail']['repeat'] = image.repeat

            if image.keep_state:
                event['front']['keep_state'] = True
                event['tail']['keep_state'] = True

            event['front']['duration'] = image.duration
            event['tail']['duration'] = image.duration

            event['front']['image'] = image.image_front
            event['tail']['image'] = image.image_tail

            animation = Event({'animation':event, 'id':0}, self._num_led, self._global_brightness)
            self._anim_queue.insert(0, animation)
            self._interrupt = True
            return 'success'
        except Exception as e:
            rospy.logerr(f'{e}')
            return 'failed'


    def _set_brightness(self, value):
        '''rosservice callback for /set_brightbness'''
        self._controller.set_brightness(value.brightness)
        return f'brightness: {value.brightness}'


    def _set_panel_state(self, panels):
        '''rosservice callback for /set_brightbness'''
        self._controller.set_panel_state(0, panels.front)
        self._controller.set_panel_state(1, panels.tail)


if __name__ == '__main__':
    try:
        lights = LightsNode()
        lights.run()
    except Exception as e:
        rospy.logerr(f'panther_lights error: {e}')
        exit(1)