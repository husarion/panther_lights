from .animation import Animation
import numpy as np

class BatteryAnimation(Animation):
    
    ANIMATION_NAME = 'battery'
    def __init__(self, anim_yaml, num_led, global_brightness, panel):
        '''battery animation class'''
        super().__init__(anim_yaml, num_led, global_brightness, panel)

        color = 0x00FF00
        if 'color' in anim_yaml.keys():
            color = anim_yaml['color']

        self._r = (np.uint32(color) >> 16) & (0x0000FF)
        self._g = (np.uint32(color) >>  8) & (0x0000FF)
        self._b = (np.uint32(color)      ) & (0x0000FF)


        self._percent_point = None
        self._empty_frame = np.zeros(num_led)
        self._full_frame = np.copy(self._empty_frame)

        self._i = 0
        self._cycle_steps = 30
        self._bright_proportion = 0.9
        
        self._dark_time = self._bright_proportion * self._duration
        self._bright_sleep_time = (self._bright_proportion * self._duration) / self._cycle_steps

        self._sleep_time = self._bright_sleep_time
    

    def __call__(self):
        '''returns new frame'''
        if self._i < self._cycle_steps:
            self._sleep_time = self._bright_sleep_time
            sin_val = np.sin((self._i / self._cycle_steps) * np.pi)
            r = np.uint32((self._r * sin_val)) << 16
            g = np.uint32((self._g * sin_val)) << 8
            b = np.uint32((self._b * sin_val))
            color = r + g + b
            self._full_frame[self._percent_point] = color
            self._full_frame[self._num_led-self._percent_point-1] = color
            self._i += 1
            return self._full_frame

        if self._i <= self._cycle_steps:
            self._sleep_time = self._dark_time
            self._i += 1
            return self._empty_frame

        raise Animation.AnimationFinished


    @property
    def sleep_time(self):
        '''returns time needed to sleep in thread between frames'''
        return self._sleep_time


    def reset(self):
        '''restets animation to it's initial state'''
        self._i = 0


    def param(self, val):
        '''sets animations param'''
        val = 1-val
        self._percent_point = int(val/2 * (self._num_led-1))
    param = property(None, param)