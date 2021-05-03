import logging
from threading import Thread, Event, Lock
from enum import Enum
import sys
import time
from queue import Queue, PriorityQueue
import numpy as np

from driver import VirtualLEDController, HardwareAPA102Controller
from animation import *

import yaml 

import signal
import sys

import os

class PantherLightsThread(Thread):

    def __init__(self,
                 queue,
                 time_step=0.01,
                 num_leds=48,
                 led_switch_pin=20,
                 led_power_pin = 26):

        '''Initialize PantherLights thread'''
        super().__init__(name="panther_lights_thread")

        self._time_step = time_step
        self._num_leds = num_leds

        self._led = VirtualLEDController(num_leds=self._num_leds, panel_count=2)
        # self._led = HardwareAPA102Controller(num_leds=num_leds, panel_count=2, led_switch_pin=led_switch_pin, led_power_pin=led_power_pin)

        self._is_running = True
        self._queue = queue
        self._background_animation = None
        self._animation_queue = PriorityQueue()

    @property
    def panel_state(self, panel_num):
        return self._panels[panel_num]

    def enable_panel(self, panel_num):
        self._panels[panel_num] = True

    def disable_panel(self, panel_num):
        self._panels[panel_num] = True

    def join(self):
        self._is_running = False
        del self._led
   
    def run(self):
        logging.info("Thread %s: started!", self._name)

        src_path = os.path.dirname(__file__)
        conf_path = os.path.relpath('../../config/led_conf.yaml', src_path)

        importer = LEDConfigImporter(conf_path)
        animations = [importer.get_animation_by_id(25), importer.get_animation_by_name('TURN-LEFT')]

        while self._is_running:
            start_time = time.time()

            if animations:
                frame = animations[0](0)
                if frame is not None:
                    self._led.set_panel(0, frame)

                frame = animations[0](1)
                if frame is not None:
                    self._led.set_panel(1, frame)
                else:
                    animations.remove(animations[0])

            finish_time = time.time()
            time.sleep(self._time_step - (finish_time - start_time))




panther_lights = PantherLightsThread(None)

def signal_handler(signum, frame):
    signal.signal(signum, signal.SIG_IGN)
    panther_lights.join()
    sys.exit(0)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)

    panther_lights.start()
    time.sleep(10)
    panther_lights.join()