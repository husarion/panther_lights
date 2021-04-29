import logging
from threading import Thread, Event, Lock
from enum import Enum
import sys
import time
from math import ceil
from queue import Queue, PriorityQueue
import numpy as np

from driver import VirtualLEDController, HardwareAPA102Controller
from animation import *


import signal
import sys

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
        a = Slide(0, 5, self._time_step, self._num_leds, 0, 47, [255,255,0])
        # b = Slide(0, 5, self._time_step, self._num_leds, 10, 30, [255,255,0])
        while self._is_running:
            # if not self._animation_queue.empty():

            # else:
            # start = time.time()
            frame = a()
            if frame is not None:
                self._led.set_panel(0, frame)

            # frame = b()
            # if frame is not None:
            #     self._led.set_panel(1, frame)
            # print(time.time() - start)
            time.sleep(self._time_step)




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