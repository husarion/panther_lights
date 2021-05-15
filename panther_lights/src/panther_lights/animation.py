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

        animation_keywords = ['duration', 'color']
        if not set(animation_keywords).issubset(anim_yaml.keys()):
            raise Animation.AnimationYAMLError('No animation specific parameters in YAML')

        self._duration = anim_yaml['duration']

        self._color = anim_yaml['color']
        if type(self._color) == int:
            r = (self._color >> 16) & (0x0000FF)
            g = (self._color >>  8) & (0x0000FF)
            b = (self._color)       & (0x0000FF)
            self._color = [r, g, b]


        if 'repeat' in anim_yaml.keys():
            self._loops = anim_yaml['repeat']
            if self._loops <= 0:
                raise Animation.AnimationYAMLError('Repeat count can\'t be negative nor equal to zero.')
        else:
            self._loops = 1

        if 'keep_state' in anim_yaml.keys():
            self._keep_state = bool(anim_yaml['keep_state'])
        else:
            self._keep_state = False

        if 'wait_after_sequence' in anim_yaml.keys():
            self._wait_after_sequence = anim_yaml['wait_after_sequence']
            if self._wait_after_sequence < 0:
                raise Animation.AnimationYAMLError('Wait_after_sequence count can\'t be negative.')
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


    def __str__(self):
        return f'''
                    duration: {self._duration}
                    color: {self._color}
                    repeat: {self._loops}
                    keep_state: {self._keep_state}
                    wait_after_sequence: {self._wait_after_sequence}
                    brightness: {self._brightness}
                '''



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
        return self._frame_counter / self._time_to_live



class Slide(Animation):
    ANIMATION_ID = 0
    def __init__(self, anim_yaml, num_led, time_step, global_brightness):
        super().__init__(anim_yaml, num_led, time_step, global_brightness)

        animation_keywords = ['range']
        if not set(animation_keywords).issubset(anim_yaml.keys()):
            raise Animation.AnimationYAMLError(f'No {set(animation_keywords) - set(yaml.keys())} in {anim_yaml}')

        self._start_point = anim_yaml['range'][0]
        self._end_point = anim_yaml['range'][1]

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



class Delay(Animation):
    ANIMATION_ID = -1
    def __init__(self, yaml, num_led, time_step, global_brightness):
        super().__init__(yaml, num_led, time_step, global_brightness)


    def _animation_callback(self):
        return None



class Executor:

    BASIC_ANIMATIONS = {
        Delay.ANIMATION_ID : Delay,
        Slide.ANIMATION_ID : Slide
    }

    def __init__(self, event_yaml, num_led, time_step, global_brightness):
        global_keywords = ['id', 'name', 'animation']
        if not set(global_keywords).issubset(event_yaml.keys()):
            raise LEDConfigImporter.LEDConfigImporterError(f'No {set(global_keywords) - set(event_yaml.keys())} in {event_yaml}')

        self._id = event_yaml['id']
        self._name = event_yaml['name']
        if 'interrupting' in event_yaml.keys():
            self._interrupting = event_yaml['interrupting']
        else:
            self._interrupting = False

        animation_keywords = ['front', 'tail']
        if not set(animation_keywords).issubset(event_yaml['animation'].keys()):
            if 'both' not in event_yaml['animation'].keys():
                raise LEDConfigImporter.LEDConfigImporterError(f'No {(set(animation_keywords + ["both"])) - set(event_yaml["animation"].keys())} in {event_yaml}')
            else:
                if 'animation_id' not in event_yaml['animation']['both'].keys():
                    raise LEDConfigImporter.LEDConfigImporterError(f'No id for light animations in {event_yaml}')
                self._front_animation = Executor.BASIC_ANIMATIONS[event_yaml['animation']['both']['animation_id']](event_yaml['animation']['both'], num_led, time_step, global_brightness)
                self._tail_animation = copy.copy(self._front_animation)
        else:
            if 'animation_id' not in event_yaml['animation']['front'].keys():
                raise LEDConfigImporter.LEDConfigImporterError(f'No id for front animation in {event_yaml}')
            self._front_animation = Executor.BASIC_ANIMATIONS[event_yaml['animation']['front']['animation_id']](event_yaml['animation']['front'], num_led, time_step, global_brightness)

            if 'animation_id' not in event_yaml['animation']['tail'].keys():
                raise LEDConfigImporter.LEDConfigImporterError(f'No id for tail animation in {event_yaml}')
            self._tail_animation = Executor.BASIC_ANIMATIONS[event_yaml['animation']['tail']['animation_id']](event_yaml['animation']['tail'], num_led, time_step, global_brightness)


    def __call__(self, panel):
        if panel == 0:
            return self._front_animation()
        if panel == 1:
            return self._tail_animation()


    def __str__(self):
        return f'''
                front:
                    {self._front_animation}
                tail:
                    {self._tail_animation}
                '''


    def reset(self):
        self._front_animation.reset()
        self._tail_animation.reset()


    @property
    def id(self):
        return self._id
        

    @property
    def brightness(self):
        return self._front_animation.brightness()


    @property
    def interrupting(self):
        return self._interrupting


    @property
    def percent_done(self):
        return min(self._front_animation.percent_done, self._tail_animation.percent_done)
        


class LEDConfigImporter:
    class LEDConfigImporterError(Exception):
        def __init__(self, message='YAML keyword error'):
            self.message = message
            super().__init__(self.message)


    def __init__(self, yaml_path):

        self._yaml = yaml.load(open(yaml_path,'r'), Loader=yaml.Loader)

        self._yaml_path = yaml_path

        global_keywords = ['global_brightness', 'num_led', 'time_step', 'led_switch_pin', 'led_power_pin', 'event_animations_files']
        if not set(global_keywords).issubset(self._yaml.keys()):
            raise LEDConfigImporter.LEDConfigImporterError(f'No {set(global_keywords) - set(self._yaml.keys())} in {self._yaml_path}')

        self._global_brightness = self._yaml['global_brightness']
        self._num_led = self._yaml['num_led']
        self._time_step = self._yaml['time_step']
        self._led_switch_pin = self._yaml['led_switch_pin']
        self._led_power_pin = self._yaml['led_power_pin']
        self._imported_animations = {}
        self._added_animations = {}

        for file in self._yaml['event_animations_files']:
            if os.path.isabs(file):
                self._import_file(file)
            else:
                src_path = os.path.dirname(__file__)
                conf_path = os.path.join(src_path, f'../../config/{file}')
                self._import_file(conf_path)


    def _import_file(self, file):
        event_yaml = yaml.load(open(file,'r'), Loader=yaml.Loader)

        if 'event' not in event_yaml.keys():
            raise LEDConfigImporter.LEDConfigImporterError(f'No \'event\' in {file}.')

        id_map = {i : event_yaml['event'][i]['id'] for i in range(len(event_yaml['event']))}
        if len(id_map.values()) != len(set(id_map.values())):
            raise LEDConfigImporter.LEDConfigImporterError(f'Events\' IDs in {file} aren\'t uniqueue.')

        name_map = {i : event_yaml['event'][i]['name'] for i in range(len(event_yaml['event']))}
        if len(name_map.values()) != len(set(name_map.values())):
            raise LEDConfigImporter.LEDConfigImporterError(f'Events\' names in {file} aren\'t uniqueue.')

        if set(id_map.values()) & set(self._imported_animations.keys()):
            raise LEDConfigImporter.LEDConfigImporterError(f'Events\' IDs in {file} overlap previous ID declarations.')

        for event in event_yaml['event']:
            self._imported_animations[(event['id'], event['name'])] = Executor(event, self._num_led, self._time_step, self._global_brightness)

    
    def add_animation(self, event_yaml):
        event_yaml = yaml.load(event_yaml, Loader=yaml.Loader)

        if 'event' in event_yaml.keys():
            id_map = {i : event_yaml['event'][i]['id'] for i in range(len(event_yaml['event']))}
            if len(id_map.values()) != len(set(id_map.values())):
                raise LEDConfigImporter.LEDConfigImporterError(f'Events\' IDs aren\'t uniqueue.')

            name_map = {i : event_yaml['event'][i]['name'] for i in range(len(event_yaml['event']))}
            if len(name_map.values()) != len(set(name_map.values())):
                raise LEDConfigImporter.LEDConfigImporterError(f'Events\' names aren\'t uniqueue.')

        else:
            event_yaml = event_yaml['event']

        if set(id_map.values()) & set(self._imported_animations.keys()) or set(id_map.values()) & set(self._added_animations.keys()):
            raise LEDConfigImporter.LEDConfigImporterError(f'Events\' IDs overlap previous declarations.')

        for event in event_yaml['event']:
            self._added_animations[(event['id'], event['name'])] = Executor(event, self._num_led, self._time_step, self._global_brightness)


    def _get_animation_key_by_id(self, id, animation_list):
        keys = list(animation_list.keys())
        IDs = (np.array(keys)[:,0]).astype(np.int)
        idx = np.where(IDs == id)[0]

        if len(idx):
            return keys[idx[0]]
        return (None, None)


    def _get_animation_key_by_name(self, name, animation_list):
        keys = list(self._imported_animations.keys())
        names = (np.array(keys)[:,1])
        idx = np.where(names == name)[0]

        if len(idx):
            return keys[idx[0]]
        return (None, None)


    def get_animation_by_id(self, id):
        key_file = self._get_animation_key_by_id(id, self._imported_animations)
        key_added = self._get_animation_key_by_id(id, self._added_animations)

        if key_file != (None, None):
            return copy.deepcopy(self._imported_animations[key_file])
        elif key_added != (None, None):
            return copy.deepcopy(self._added_animations[key_added])
        else:
            raise LEDConfigImporter.LEDConfigImporterError(f'Animation with ID: {id} is not defined.')


    def get_animation_by_name(self, name):
        key_file = self._get_animation_key_by_name(name, self._imported_animations)
        key_added = self._get_animation_key_by_name(name, self._added_animations)

        if key_file != (None, None):
            return copy.deepcopy(self._imported_animations[key_file])
        elif key_added != (None, None):
            return copy.deepcopy(self._added_animations[key_added])
        else:
            raise LEDConfigImporter.LEDConfigImporterError(f'Animation with name: {name} is not defined.')


    def remove_animation_by_id(self, id):
        key = self._get_animation_key_by_id(id, self._added_animations)

        if key == (None, None):
            del self._imported_animations[key]
        else:
            raise LEDConfigImporter.LEDConfigImporterError(f'Animation with id: {id} is not defined.')


    def remove_animation_by_name(self, name):
        key = self._get_animation_key_by_name(name, self._added_animations)

        if key == (None, None):
            del self._imported_animations[key]
        else:
            raise LEDConfigImporter.LEDConfigImporterError(f'Animation with name: {name} is not defined.')

    def get_animation_key(self, key=None, name=None):
        if key is not None:
            return self._get_animation_key_by_name(name, self._added_animations)

        if name is not None:
            return self._get_animation_key_by_id(id, self._added_animations)

        return (None, None)


    @property
    def global_brightness(self):
        return self._global_brightness


    @property
    def num_led(self):
        return self._num_led


    @property
    def time_step(self):
        return self._time_step


    @property
    def led_switch_pin(self):
        return self._led_switch_pin


    @property
    def led_power_pin(self):
        return self._led_power_pin
                