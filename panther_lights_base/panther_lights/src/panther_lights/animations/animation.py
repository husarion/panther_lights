import yaml
import numpy as np

class Animation:
    '''
    General class providing interface for animation implementation.
    Implements repeating animation in loop, delay between each animation, if animation keeps
    last state after finishing and tics count of full sequence.

    In order to implement own animation implement:
        __init__(self, anim_yaml, num_led, time_step, global_brightness)
        _animation_callback(self)
        reset(self)
    
    __init__ has to call:
        super().__init__(anim_yaml, num_led, time_step, global_brightness)
    
    _animation_callback is called in __call__(). This function is called only when animation is in execute loop.
    If __call__() enters wait_after_sequence _animation_callback is not called.

    _animation_callback() implements animation computation on every tick of animation and returns frame array with colors in HEX values.
    Each animation instance is destroyed after finishing all tics in every loops.

    If repeat is greater than 1 method reset() will be called. reset() has to implement bringing animation to it's initial state.

    reset() is also called externally by executor in order to restart animation if it was interrupted.
    Parent class implements reseting tick counter.
    '''
    class AnimationYAMLError(Exception):
        def __init__(self, message='YAML keyword error'):
            self.message = message
            super().__init__(self.message)


    def __init__(self, anim_yaml, num_led, time_step, global_brightness):
        '''
        Params:
            anim_yaml:              Part of events yaml file describing given animation.
            num_led:                Led count of panel. Read from led_conf.yaml
            time_step:              Time duration between animation tics. Read from led_conf.yaml
            global_brightness:      Globally set brightness of panels. Read from led_conf.yaml

        anim_yaml keys:
            duration:               Obligatory, float - positive.
                                    Duration of single animation sequence.

            wait_after_sequence:    Optional,   float - non negative, default - 0.
                                    Waits after finishing sequence. Can be used as delay between sequences in loop.

            repeat:                 Optional,   int - positive, default - 1.
                                    How many time animation will be repeated in loop.

            keep_state:             Optional,   bool, default - false.
                                    If true keeps state of last displayed frame. If false turns off panel after finishing.

            brightness:             Optional,   int - positive, less than 255, default - global_brightness.
                                    Sets panel brightness.
        '''

        self._brightness = global_brightness
        self._time_step = time_step
        self._num_led = num_led
        self._param = None

        # Check for obligatory keys
        animation_keywords = ['duration']
        if not 'duration' in anim_yaml.keys():
            raise Animation.AnimationYAMLError('no duration in parameters YAML')

        self._duration = anim_yaml['duration']
        if self._duration <= 0:
            raise Animation.AnimationYAMLError('duration has to pe positive.')

        if 'repeat' in anim_yaml.keys():
            self._loops = anim_yaml['repeat']
            if self._loops <= 0 or isinstance(self._loops, float):
                raise Animation.AnimationYAMLError('repeat count can\'t be negative, equal to zero nor float.')
        else:
            self._loops = 1

        if 'keep_state' in anim_yaml.keys():
            self._keep_state = anim_yaml['keep_state']
            if not isinstance(self._keep_state, bool) and not isinstance(self._keep_state, int) or \
                  (isinstance(self._keep_state, int) and self._keep_state not in (0,1)):
                raise Animation.AnimationYAMLError('keep_state has to be logical.')
            self._keep_state = bool(self._keep_state)
        else:
            self._keep_state = False


        if 'brightness' in anim_yaml:
            self._brightness = float(anim_yaml['brightness'])
            if not (0 < self._brightness <= 1):
                raise Animation.AnimationYAMLError('brightness has match boundaries 0 < brightness <= 1.')
            self._brightness = int(round(self._brightness * 255))

        self._tick = 0
        self._max_tics = self._duration / self._time_step

        self._current_loop = 0

        self._frame = np.zeros((1,self._num_led))

        self._last_frame = False


    def __call__(self):
        '''
        Interface for calling animation.
        Calls animation every tick. Implements running in loop and delays between animations.
        Returns:
            while animation not finished:
                list of HEX colour values of panel LED.
            after animation finishes:
                None
        '''

        if self._tick >= self._max_tics:
            self._current_loop += 1
            self.reset()
            if self._current_loop == self._loops:
                self._last_frame = True
            if self._keep_state:
                return self._frame
            else:
                return np.zeros((self._num_led)).astype(np.uint32)
        
        if self._last_frame:
            return None
        
        self._frame = self._animation_callback()
        self._tick += 1
        return self._frame




    def _animation_callback(self):
        '''
        Callback used in __call__ to implement animation behaviour on each tick.
        Returns:
            list of HEX colour values of panel LEDs.
        '''
        raise NotImplementedError


    def reset(self):
        '''
        Restets animation to it's state at beginning of each loop
        '''
        self._tick = 0
        self._last_frame = False

    def param(self, val):
        raise NotImplementedError
    param = property(None, param)


    @property
    def brightness(self):
        return self._brightness


    @property
    def percent_done(self):
        return ((self._tick / self._max_tics) + self._current_loop) / self._loops
