import logging
from threading import Thread, Event, Lock
from enum import Enum
import sys
import time
from math import ceil
from queue import Queue
import numpy as np

class AnimationInterface:
    def __init__(self, start_time, duration):
        pass

    def __call__(self):
        pass



class Slide(AnimationInterface):
    def __init__(self, delay, duration, time_step, pixel_cnt, start_point, end_point, color):
        self._duration = duration
        self._pixel_cnt = pixel_cnt
        self._start_point = start_point
        self._end_point = end_point

        self._direction = np.sign(self._end_point - self._start_point)

        self._frame = np.zeros((3,self._pixel_cnt))
        self._frame[:,start_point] = np.array(color)

        self._frame_counter = -delay
        self._time_to_life = duration / time_step

        self._update_rate = int(self._time_to_life / np.abs(start_point - end_point + 1))
        print(self._time_to_life, self._update_rate, self._time_to_life / np.abs(start_point - end_point + 1))


    def __call__(self):
        self._frame_counter += 1
        if self._frame_counter > 0 and (self._frame_counter % self._update_rate) == 0:
            self._frame = np.roll(self._frame, self._direction, axis=1)

        if self._frame_counter >= self._time_to_life:
            return None

        return self._frame.astype(np.uint8)