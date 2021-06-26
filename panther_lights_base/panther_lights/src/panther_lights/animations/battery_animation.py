from .animation import Animation
import numpy as np

class BatteryAnimation(Animation):
    ANIMATION_NAME = 'battery'
    def __init__(self, anim_yaml, num_led, time_step, global_brightness):
        super().__init__(anim_yaml, num_led, time_step, global_brightness)

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
    

    def _animation_callback(self):
        return self._full_frame

        # if self._tick >= self._frame_counter_threshold:
        #     self._frame = self._img[self._frame_counter,:]
        #     self._frame_counter_threshold += self._frame_tics
        #     self._frame_counter += 1

        
    def param(self, val):
        self._percentage = val

        point = int(self._percent_steps * self._num_led)
        self._full_frame[point] = 0xFFFFFF
        self._full_frame[-point] = 0xFFFFFF
 


    def reset(self):
        super().reset()