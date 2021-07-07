from .animation import Animation
import numpy as np

class BatteryAnimation(Animation):
    
    ANIMATION_NAME = 'battery'
    def __init__(self, anim_yaml, num_led, global_brightness, panel):
        '''battery animation class'''
        super().__init__(anim_yaml, num_led, global_brightness, panel)

        self._num_led = num_led
        mirror_num_led = num_led
        if mirror_num_led % 2:
            mirror_num_led -= 1
            self._percent_steps = 1 / (mirror_num_led/2 + 1)
        else:
            self._percent_steps = 1 / (mirror_num_led/2)

        self._empty_frame = np.zeros(num_led)
        self._full_frame = np.copy(self._empty_frame)

        percentage_indicator = num_led // 2
    

    def __call__(self):
        '''returns new frame'''
        while self._i < self._img_y:
            frame = self._img[self._i,:]
            self._i += 1
            return frame
        raise Animation.AnimationFinishedError


    @property
    def sleep_time(self):
        '''returns time needed to sleep in thread between frames'''
        return self._frame_time


    def reset(self):
        '''restets animation to it's initial state'''
        self._i = 0


    def param(self, val):
        '''sets animations param'''
        self._percentage = val
        point = int(self._percent_steps * self._num_led)
        self._full_frame[point] = 0xFFFFFF
        self._full_frame[-point] = 0xFFFFFF