
class Animation:

    class AnimationYAMLError(Exception):
        '''basic animation import error'''
        def __init__(self, message='YAML keyword error'):
            self.message = message
            super().__init__(self.message)


    class AnimationFinished(Exception):
        '''exception indication that animation finished'''
        def __init__(self, message='YAML keyword error'):
            self.message = message
            super().__init__(self.message)


    def __init__(self, anim_yaml, num_led, global_brightness, panel):
        '''animation base class'''
        self._brightness = global_brightness
        self._num_led = num_led
        self._panel = panel
        self._param = None

        # Check for obligatory keys
        animation_keywords = ['duration']
        if not animation_keywords in anim_yaml.keys():
            raise Animation.AnimationYAMLError('no duration in parameters YAML')

        self._duration = anim_yaml['duration']
        if self._duration <= 0:
            raise Animation.AnimationYAMLError('duration has to pe positive')

        if 'repeat' in anim_yaml.keys():
            self._loops = anim_yaml['repeat']
            if self._loops <= 0 or isinstance(self._loops, float):
                raise Animation.AnimationYAMLError('repeat count can\'t be negative, equal to zero nor float')
        else:
            self._loops = 1

        if 'brightness' in anim_yaml:
            self._brightness = float(anim_yaml['brightness'])
            if not (0 < self._brightness <= 1):
                raise Animation.AnimationYAMLError('brightness has match boundaries 0 < brightness <= 1')
            self._brightness = int(round(self._brightness * 255))


    def __call__(self):
        '''returns new frame'''
        raise NotImplementedError


    @property
    def sleep_time(self):
        '''returns time needed to sleep in thread between frames'''
        raise NotImplementedError


    def reset(self):
        '''restets animation to it's initial state'''
        raise NotImplementedError


    def param(self, val):
        '''sets animations param'''
        raise NotImplementedError
    param = property(None, param)


    @property
    def brightness(self):
        '''returns animation brightness'''
        return self._brightness


    @property
    def loops(self):
        '''returns how many times animation repeats'''
        return self._loops


    @property
    def keep_state(self):
        '''returns if animation keeps it's last state'''
        return self._keep_state


    @property
    def duration(self):
        '''returns animation's single loop duration'''
        return self._duration


    @property
    def num_led(self):
        '''returns panel's LED count'''
        return self._num_led


    @property
    def panel(self):
        '''returns which panel animation is assigned'''
        return self._panel


    @panel.setter
    def panel(self, val):
        '''assigns animation to panel'''
        self._panel = val
