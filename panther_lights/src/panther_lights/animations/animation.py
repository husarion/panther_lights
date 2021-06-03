import yaml
import numpy as np

class Animation:
    class AnimationYAMLError(Exception):
        def __init__(self, message='YAML keyword error'):
            self.message = message
            super().__init__(self.message)


    def __init__(self, anim_yaml, num_led, time_step, global_brightness):

        self._brightness = global_brightness
        self._time_step = time_step
        self._num_led = num_led

        animation_keywords = ['duration']
        if not 'duration' in anim_yaml.keys():
            raise Animation.AnimationYAMLError('no duration in parameters YAML')

        self._duration = anim_yaml['duration']
        if self._duration <= 0:
            raise Animation.AnimationYAMLError('duration has to pe positive.')

        if 'repeat' in anim_yaml.keys():
            self._loops = anim_yaml['repeat']
            if self._loops <= 0:
                raise Animation.AnimationYAMLError('repeat count can\'t be negative nor equal to zero.')
        else:
            self._loops = 1

        if 'keep_state' in anim_yaml.keys():
            self._keep_state = bool(anim_yaml['keep_state'])
        else:
            self._keep_state = False

        if 'wait_after_sequence' in anim_yaml.keys():
            self._wait_after_sequence = anim_yaml['wait_after_sequence']
            if self._wait_after_sequence < 0:
                raise Animation.AnimationYAMLError('wait_after_sequence count can\'t be negative.')
        else:
            self._wait_after_sequence = 0

        if 'brightness' in anim_yaml:
            self._brightness = anim_yaml['brightness']

        self._frame_counter = 0
        self._sequence_time = self._duration / self._time_step

        self._wait_frames_counter = 0
        self._wait_frames = self._wait_after_sequence / self._time_step

        self._current_loop = 0

        self._frame = np.zeros((3,self._num_led))

        self._is_waiting = False
        self._last_frame = False


    def __call__(self):
        if self._wait_frames > 0:
            if not self._last_frame:
                if not self._is_waiting:
                    if self._frame_counter >= self._sequence_time:
                        self._frame_counter = 0
                        self._is_waiting = True

                    self._frame = self._animation_callback()
                    self._frame_counter += 1
                    return self._frame

                else:
                    if self._wait_frames_counter >= self._wait_frames:
                        self._is_waiting = False
                        self._wait_frames_counter = 0
                        self._current_loop += 1
                    self._wait_frames_counter += 1
                    if self._current_loop == self._loops:
                        self._last_frame = True

                    if self._keep_state:
                        return self._frame
                    else:
                        return np.zeros((3,self._num_led))

            return None

        else:
            if self._frame_counter >= self._sequence_time:
                self._frame_counter = 0
                self._current_loop += 1
                self._is_waiting = True
                if self._current_loop == self._loops:
                    self._last_frame = True
                if self._keep_state:
                    return self._frame
                else:
                    return np.zeros((3,self._num_led))
            
            if self._last_frame:
                return None
            
            self._frame = self._animation_callback()
            self._frame_counter += 1
            return self._frame




    def _animation_callback(self):
        raise NotImplementedError


    def reset(self):
        self._frame = np.zeros((3,self._num_led))
        self._frame_counter = 0
        self._current_loop = 0
        self._wait_frames_counter = 0
        self._is_waiting = False
        self._last_frame = False


    @property
    def brightness(self):
        return self._brightness


    @property
    def percent_done(self):
        return ((self._frame_counter / self._sequence_time) + self._current_loop) / self._loops