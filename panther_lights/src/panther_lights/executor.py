import numpy as np
import copy
import yaml
import os

from .animations import *

class Executor:

    class ExecuterError(Exception):
        def __init__(self, message='YAML keyword error'):
            self.message = message
            super().__init__(self.message)


    def __init__(self, event_yaml, num_led, time_step, global_brightness):

        '''
        Params:
            event_yaml:             Part of events yaml file describing given animation.
            num_led:                Led count of panel. Read from led_conf.yaml
            time_step:              Time duration between animation tics. Read from led_conf.yaml
            global_brightness:      Globally set brightness of panels. Read from led_conf.yaml

        anim_yaml keys:
            id:                     Obligatory, int - positive.
                                    Animation ID used by rosservice.

            name:                   Obligatory, string
                                    Animation name used by rosservice.

            interrupting:           Optional, bool, default - false.
                                    Allows animation to interrupt currently running animation.

            both | [front, tail]:   Obligatory.
                                    If specified 'both' the same animation will be applied to front and tail panel.
                                    If font and tail specified different animations will be used for both panels.
        '''

        if 'interrupting' in event_yaml.keys():
            self._interrupting = event_yaml['interrupting']
        else:
            self._interrupting = False


        if 'both' in event_yaml['animation'].keys():
            self._front_animation = self._match_animation_type(event_yaml['animation']['both'], num_led, time_step, global_brightness)
            self._tail_animation = copy.copy(self._front_animation)

        elif 'front' in event_yaml['animation'].keys() and 'tail' in event_yaml['animation'].keys():
            self._front_animation = self._match_animation_type(event_yaml['animation']['front'], num_led, time_step, global_brightness)
            self._tail_animation = self._match_animation_type(event_yaml['animation']['tail'], num_led, time_step, global_brightness)
        else:
            raise Executor.ExecuterError(f'no {(set(["front", "tail", "both"])) - set(event_yaml["animation"].keys())} in {event_yaml}')


    def __call__(self, panel):
        '''
        Compute animation for each panel and return list of colors in hex values.
        '''
        if panel == 0:
            return self._front_animation()
        if panel == 1:
            return self._tail_animation()

    
    def _match_animation_type(self, animation_yaml, *args):
        if 'animation_name' in animation_yaml.keys():
            return BASIC_ANIMATIONS[animation_yaml['animation_name']](animation_yaml, *args)
        elif 'image' in animation_yaml.keys():
            return BASIC_ANIMATIONS['image_animation'](animation_yaml, *args)
        else:
            raise Executor.ExecuterError(f'no matching animation type in {animation_yaml}')


    def reset(self):
        self._front_animation.reset()
        self._tail_animation.reset()
        

    @property
    def brightness(self):
        return self._front_animation.brightness()


    @property
    def interrupting(self):
        return self._interrupting


    @property
    def percent_done(self):
        return min(self._front_animation.percent_done, self._tail_animation.percent_done)