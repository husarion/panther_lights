#!/usr/bin/env python3

import queue
import time
import yaml
import copy
import os

import rospy
from std_srvs.srv import Empty
from panther_lights.driver import VirtualLEDController, HardwareAPA102Controller
from panther_lights.panther_thread import PantherLightsAnimationExecutorThread, AnimationLock
from panther_lights.led_config_importer import LEDConfigImporter
from panther_lights.executor import Executor
from panther_lights.srv import Brightness, Animation, ImageAnimation, PanelState


class LightsNode:
    def __init__(self,
                 config_path=None):

        if config_path is None:
            config_path = '../config/led_conf.yaml'

        src_path = os.path.dirname(os.path.abspath(__file__))
        conf_path = os.path.join(src_path, config_path)
        conf_file = open(conf_path,'r')
        conf_yaml = yaml.load(conf_file, Loader=yaml.Loader)
        conf_file.close()

        self._led_config_importer = LEDConfigImporter(conf_yaml)

        self._time_step = self._led_config_importer.time_step
        self._num_led = self._led_config_importer.num_led
        self._global_brightness = self._led_config_importer.time_step

        self._driver = None


        panther_driver_type = os.getenv('PANTHER_LIGHTS_DRIVER')
        if panther_driver_type == 'PLT_GUI':
            try:
                import matplotlib
                import matplotlib.pyplot as plt
                import warnings
            except ImportError as e:
                raise ImportError(f'unable to import matplotlib. Make sure you have installed matplotlib. {e}')
            self._driver = VirtualLEDController(num_led=self._num_led)

        elif panther_driver_type == 'APA102':
            try:
                import RPi.GPIO as GPIO
                from apa102_pi.driver import apa102 
            except ImportError:
                raise ImportError('no hardware specific packages installed. Make sure you are running this node on Raspberry pi.')
            self._driver = HardwareAPA102Controller(num_led=self._num_led,
                                                    brightness=self._global_brightness)

        else:
            raise OSError(f'\'{panther_driver_type}\' - unknown driver type')


        self._queue = queue.Queue()
        self._emergency_lock = AnimationLock()
        self._interrupt_lock = AnimationLock()
        self._exector = PantherLightsAnimationExecutorThread(self._queue, self._emergency_lock, self._interrupt_lock,
                                                             self._driver, self._time_step)


        self._system_indicators = True
        self._emergency_animation_id = (-1,-2,-3)


        rospy.init_node('lights_node')

        self._rate = rospy.Rate(10)

        self._set_lights_by_name_service = rospy.Service('set_lights', Animation, self._set_lights)
        self._set_image_animation = rospy.Service('set_imgae_animation', ImageAnimation, self._set_image_animation)
        self._set_brightness_service = rospy.Service('set_brightness', Brightness, self._set_brightness)
        self._set_panel_state_service = rospy.Service('set_panel_state', PanelState, self._set_panel_state)


    def run(self):
        self._exector.start()

        rospy.loginfo('panther_lights started')
        while not rospy.is_shutdown():
            self._rate.sleep()
            
        self._exector.join()
        del self._driver
        

    def _set_lights(self, anim):
        try:
            if anim.id != 0 and not anim.name:
                animation = self._led_config_importer.get_animation(id=anim.id)
            elif anim.id == 0 and anim.name:
                animation = self._led_config_importer.get_animation(name=anim.name)
            else:
                animation = self._led_config_importer.get_animation(id=anim.id, name=anim.name)
        except Exception as e:
            return f'failed'

        if animation.id in self._emergency_animation_id:
            self._emergency_lock.set_executor(animation)
        elif animation.interrupting:
            self._interrupt_lock.set_executor(animation)
        else:
            self._queue.put(animation)

        return f'success'


    def _set_image_animation(self, image):

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

            animation = Executor({'animation': event}, self._num_led, self._time_step, self._global_brightness)
            self._interrupt_lock.set_executor(animation)
            return 'success'
        except Exception as e:
            return 'failed'


    def _set_brightness(self, value):
        self._driver.set_brightness(value.brightness)
        return f'brightness: {value.brightness}'


    def _set_panel_state(self, panels):
        self._driver.set_panel_state(0, panels.front)
        self._driver.set_panel_state(1, panels.tail)



if __name__ == '__main__':
    try:
        lights = LightsNode()
        lights.run()
    except Exception as e:
        rospy.logerr(f'panther_lights error: {e}')
        exit(1)