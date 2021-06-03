from .animation import Animation
import numpy as np

class Slide(Animation):
    ANIMATION_NAME = 'slide'
    def __init__(self, anim_yaml, num_led, time_step, global_brightness):
        super().__init__(anim_yaml, num_led, time_step, global_brightness)

        self._start_point = 0
        self._end_point = num_led

        self._color = [255,255,255]

        self._direction = np.sign(self._end_point - self._start_point)
        self._frame[:,self._start_point] = np.array(self._color)
        self._update_rate = np.floor(self._sequence_time / np.abs(self._start_point - self._end_point + 1))


    def _animation_callback(self):
        if (self._frame_counter % self._update_rate) == 0:
            self._frame = np.roll(self._frame, self._direction, axis=1)

        return self._frame.astype(np.uint8)


    def reset(self):
        super().reset()
        self._frame[:,self._start_point] = np.array(self._color)