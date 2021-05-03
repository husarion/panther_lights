import numpy as np
import copy
import yaml
import os

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
        if not set(animation_keywords).issubset(anim_yaml.keys()):
            raise Animation.AnimationYAMLError('No animation specific parameters in YAML')

        self._duration = anim_yaml['duration']

        if 'delay' in anim_yaml:
            self._delay = anim_yaml['delay']

        if 'brightness' in anim_yaml:
            self._brightness = anim_yaml['brightness']

        self._frame_counter = -self._delay
        self._time_to_life = self._duration / self._time_step

        self._frame = np.zeros((3,self._num_led))


    def __call__(self, last_frame):
        pass

    def reset(self):
        self._frame_counter = -self._delay

    @property
    def brightness(self):
        return self._brightness



class Slide(Animation):
    ANIMATION_ID = 0
    ANIMATION_NAME = 'Slide'
    def __init__(self, anim_yaml, num_led, time_step, global_brightness):
        super().__init__(anim_yaml, num_led, time_step, global_brightness)

        animation_keywords = ['range', 'color']
        if not set(animation_keywords).issubset(anim_yaml.keys()):
            raise Animation.AnimationYAMLError(f'No {set(animation_keywords) - set(yaml.keys())} in {anim_yaml}')

        self._start_point = anim_yaml['range'][0]
        self._end_point = anim_yaml['range'][1]
        self._color = anim_yaml['color']

        self._direction = np.sign(self._end_point - self._start_point)

        self._frame[:,self._start_point] = np.array(self._color)

        self._update_rate = int(self._time_to_life / np.abs(self._start_point - self._end_point + 1))


    def __call__(self):
        self._frame_counter += 1
        if self._frame_counter > 0 and (self._frame_counter % self._update_rate) == 0:
            self._frame = np.roll(self._frame, self._direction, axis=1)

        if self._frame_counter >= self._time_to_life:
            return None

        return self._frame.astype(np.uint8)



class Delay(Animation):
    ANIMATION_ID = -1
    ANIMATION_NAME = 'Delay'
    def __init__(self, yaml, num_led, time_step, global_brightness):
        super().__init__(yaml, num_led, time_step, global_brightness)

    def __call__(self):
        return None

    def overwrite_parameters(self, local_yaml):
        super().overwrite_parameters(local_yaml)


class AdvAnimation:

    BASIC_ANIMATIONS = {
        Delay.ANIMATION_ID : Delay,
        Slide.ANIMATION_ID : Slide
    }

    def __init__(self, anim_yaml, num_led, time_step, global_brightness):
        global_keywords = ['id', 'name', 'priority', 'animation']
        if not set(global_keywords).issubset(anim_yaml.keys()):
            raise LEDConfigImporter.LEDConfigImporterError(f'No {set(global_keywords) - set(anim_yaml.keys())} in {anim_yaml}')

        self._id = anim_yaml['id']
        self._name = anim_yaml['name']
        self._priority = anim_yaml['priority']

        animation_keywords = ['front', 'tail']
        if not set(animation_keywords).issubset(anim_yaml['animation'].keys()):
            if 'both' not in anim_yaml['animation'].keys():
                raise LEDConfigImporter.LEDConfigImporterError(f'No {(set(animation_keywords + ["both"])) - set(anim_yaml["animation"].keys())} in {anim_yaml}')
            else:
                if 'id' not in anim_yaml['animation']['both'].keys():
                    raise LEDConfigImporter.LEDConfigImporterError(f'No id for light animations in {anim_yaml}')
                self._front_animation = AdvAnimation.BASIC_ANIMATIONS[anim_yaml['animation']['both']['id']](anim_yaml['animation']['both'], num_led, time_step, global_brightness)
                self._tail_animation = copy.copy(self._front_animation)
        else:
            if 'id' not in anim_yaml['animation']['front'].keys():
                raise LEDConfigImporter.LEDConfigImporterError(f'No id for front animation in {anim_yaml}')
            self._front_animation = AdvAnimation.BASIC_ANIMATIONS[anim_yaml['animation']['front']['id']](anim_yaml['animation']['front'], num_led, time_step, global_brightness)

            if 'id' not in anim_yaml['animation']['tail'].keys():
                raise LEDConfigImporter.LEDConfigImporterError(f'No id for tail animation in {anim_yaml}')
            self._tail_animation = AdvAnimation.BASIC_ANIMATIONS[anim_yaml['animation']['tail']['id']](anim_yaml['animation']['tail'], num_led, time_step, global_brightness)


    def __call__(self, panel):
        if panel == 0:
            return self._front_animation()
        if panel == 1:
            return self._tail_animation()

    def brightness(self, panel):
        return self._front_animation.brightness()


class LEDConfigImporter:

    class LEDConfigImporterError(Exception):
        def __init__(self, message='YAML keyword error'):
            self.message = message
            super().__init__(self.message)

    class Executor:
        def __init__(self, executor_list):
            self._executor = executor
            self._frame = None

        def __call__(self):
            pass



    def __init__(self, yaml_path):

        self._yaml = yaml.load(open(yaml_path,'r'), Loader=yaml.Loader)

        self._yaml_path = yaml_path

        global_keywords = ['global_brightness', 'num_led', 'time_step', 'animation_files']
        if not set(global_keywords).issubset(self._yaml.keys()):
            raise LEDConfigImporter.LEDConfigImporterError(f'No {set(global_keywords) - set(yaml.keys())} in {self._yaml_path}')

        self._global_brightness = self._yaml['global_brightness']
        self._num_led = self._yaml['num_led']
        self._time_step = self._yaml['time_step']
        self._imported_animations = {}

        for file in self._yaml['animation_files']:
            if os.path.isabs(file):
                self._import_file(file)
            else:
                src_path = os.path.dirname(__file__)
                conf_path = os.path.relpath(f'../../config/{file}', src_path)
                self._import_file(conf_path)


    def _import_file(self, file):
        anim_yaml = yaml.load(open(file,'r'), Loader=yaml.Loader)

        if 'animations' not in anim_yaml.keys():
            raise LEDConfigImporter.LEDConfigImporterError(f'No \'animations\' in {file}.')

        id_map = {i : anim_yaml['animations'][i]['id'] for i in range(len(anim_yaml['animations']))}
        if len(id_map.values()) != len(set(id_map.values())):
            raise LEDConfigImporter.LEDConfigImporterError(f'Animations\' IDs in {file} aren\'t uniqueue.')

        if set(id_map.values()) & set(self._imported_animations.keys()):
            raise LEDConfigImporter.LEDConfigImporterError(f'Animations\' IDs in {file} overlap previous ID declarations.')

        for anim in anim_yaml['animations']:
            self._imported_animations[anim['id']] = AdvAnimation(anim, self._num_led, self._time_step, self._global_brightness)



    def create_animation(self, yaml):
        pass


    def get_animation(self, id):
        return self._imported_animations[id]
                