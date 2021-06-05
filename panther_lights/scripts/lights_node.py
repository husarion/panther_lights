#!/usr/bin/env python3

import queue
import time
import yaml
import os

import rospy
from std_srvs.srv import Empty
from panther_lights.driver import VirtualLEDController, HardwareAPA102Controller
from panther_lights.panther_thread import PantherLightsAnimationExecutorThread, AnimationLock
from panther_lights.led_config_importer import LEDConfigImporter
from panther_lights.srv import Brightness, LightsID, LightsName, Animation, PanelState


class LightsNode:
    def __init__(self,
                 config_path=None,
                 use_virtual_driver=False):

        if config_path is None:
            config_path = '../config/led_conf.yaml'

        src_path = os.path.dirname(__file__)
        conf_path = os.path.join(src_path, config_path)

        self._led_config_importer = LEDConfigImporter(conf_path)

        self._time_step = self._led_config_importer.time_step
        self._num_led = self._led_config_importer.num_led
        self._global_brightness = self._led_config_importer.time_step

        self._driver = None

        if use_virtual_driver:
            try:
                import matplotlib
                import matplotlib.pyplot as plt
                import warnings
            except ImportError as e:
                raise ImportError(f'unable to import matplotlib. Make sure you have installed matplotlib. {e}')
            self._driver = VirtualLEDController(num_led=self._num_led)

        else:
            try:
                from apa102_pi.driver import apa102 
                import RPi.GPIO as GPIO
            except ImportError:
                raise ImportError('no hardware specific packages installed. Make sure you are running this node on Raspberry pi.')
            self._driver = HardwareAPA102Controller(num_led=self._num_led,
                                                    brightness=self._global_brightness)


        self._queue = queue.Queue()
        self._emergency_lock = AnimationLock()
        self._interrupt_lock = AnimationLock()
        self._exector = PantherLightsAnimationExecutorThread(self._queue, self._emergency_lock, self._interrupt_lock,
                                                             self._driver, self._time_step)


        self._system_indicators = True
        self._emergency_animation_id = (-1,-2,-3)


        rospy.init_node('lights_node')

        self._rate = rospy.Rate(10)

        self._set_lights_by_name_service = rospy.Service('set_lights_by_name', LightsName, self._set_lights_by_name)
        self._set_lights_by_id_service = rospy.Service('set_lights_by_id', LightsID, self._set_lights_by_id)
        self._disable_system_indicators_service = rospy.Service('disable_system_indicators', Empty, self._disable_system_indicators)
        self._enable_system_indicators_service = rospy.Service('enable_system_indicators', Empty, self._enable_system_indicators)
        self._set_brightness_service = rospy.Service('set_brightness', Brightness, self._set_brightness)
        self._set_panel_state_service = rospy.Service('set_panel_state', PanelState, self._set_panel_state)


    def run(self):
        self._exector.start()

        rospy.loginfo('panther_lights started')
        while not rospy.is_shutdown():
            self._rate.sleep()
            
        self._exector.join()
        del self._driver
        

    def _set_lights_by_name(self, anim):
        try:
            animation = self._led_config_importer.get_animation(name=anim.name)
        except LEDConfigImporter.LEDConfigImporterError as e:
            return f'{e}'
        except Exception as e:
            return f'failed to set lights'

        if animation.id in self._emergency_animation_id:
            self._emergency_lock.set_executor(animation)
        elif animation.interrupting:
            self._interrupt_lock.set_executor(animation)
        else:
            self._queue.put(animation)

        return f'success'


    def _set_lights_by_id(self, anim):
        try:
            animation = self._led_config_importer.get_animation(id=anim.id)
        except LEDConfigImporter.LEDConfigImporterError as e:
            return f'{e}'
        except Exception as e:
            return f'failed to set lights'

        if animation.id in self._emergency_animation_id:
            self._emergency_lock.set_executor(animation)
        elif animation.interrupting:
            self._interrupt_lock.set_executor(animation)
        else:
            self._queue.put(animation)

        return f'success'


    def _disable_system_indicators(self):
        self._system_indicators = False
        return 'system indicators: disabled'


    def _enable_system_indicators(self):
        self._system_indicators = True
        return 'system indicators: enabled'


    def _set_brightness(self, value):
        self._driver.set_brightness(value.brightness)
        return f'brightness: {value.brightness}'

    def _set_panel_state(self, panels):
        self._driver.set_panel_state(0, panels.front)
        self._driver.set_panel_state(1, panels.tail)



if __name__ == '__main__':
    try:
        lights = LightsNode(use_virtual_driver=True)
        lights.run()
    except Exception as e:
        rospy.logerr(f'panther_lights error: {e}')
        exit(1)