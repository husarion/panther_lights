from .animation import Animation
import numpy as np

class BatteryAnimation(Animation):
    ANIMATION_NAME = 'battery'
    def __init__(self, anim_yaml, num_led, time_step, global_brightness):
        super().__init__(anim_yaml, num_led, time_step, global_brightness) 
    

    def _animation_callback(self):
        pass


    def reset(self):
        super().reset()