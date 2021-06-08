import numpy as np
import copy
import yaml
import os

from .animations import *
from .executor import Executor


class LEDConfigImporter:
    class LEDConfigImporterError(Exception):
        def __init__(self, message='YAML keyword error'):
            self.message = message
            super().__init__(self.message)


    def __init__(self, yaml_path):
        '''
        Params:
            yaml_path:              Path to led_conf.yaml file containeing animation description.

        self._yaml keys:
            global_brightness:      Obligatory, int - positive, lower than 255.
                                    Sets global brightness of panels.

            num_led:                Obligatory,   int - positive.
                                    LED count for both panels.

            time_step:              Obligatory,   float - positive.
                                    Time interval between animations tics.

            event_animations_files: Obligatory,   list of strings.
                                    File names containing descriptions of animations.
        '''

        # Obligatory animations used internally by Panther
        self._husarion_animations = tuple([(10, 'E-STOP')])

        self._yaml = yaml.load(open(yaml_path,'r'), Loader=yaml.Loader)

        self._yaml_path = yaml_path

        # Check if obligatory keyword exist
        global_keywords = ['global_brightness', 'num_led', 'time_step', 'event_animations_files']
        if not set(global_keywords).issubset(self._yaml.keys()):
            raise LEDConfigImporter.LEDConfigImporterError(f'No {set(global_keywords) - set(self._yaml.keys())} in {self._yaml_path}')

        self._global_brightness = self._yaml['global_brightness']
        self._num_led = self._yaml['num_led']
        self._time_step = self._yaml['time_step']
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
        '''
        Params:
            file:                   YAML file with animations events' descriptions.

        self._yaml keys:
            event:                  Obligatory, int - positive, lower than 255.
                                    Sets global brightness of panels.

            num_led:                Obligatory,   int - positive.
                                    LED count for both panels.

            time_step:              Obligatory,   float - positive.
                                    Time interval between animations tics.

            event_animations_files: Obligatory,   list of strings.
                                    File names containing descriptions of animations.
        '''

        event_yaml = yaml.load(open(file,'r'), Loader=yaml.Loader)

        if 'event' not in event_yaml.keys():
            raise LEDConfigImporter.LEDConfigImporterError(f'No \'event\' in {file}.')

        id_map = {}
        name_map = {}

        # Create map of all names in file
        for i in range(len(event_yaml['event'])):
            if 'id' not in event_yaml['event'][i].keys():
                raise LEDConfigImporter.LEDConfigImporterError(f'No \'id\' in {event_yaml["event"][i]}.')
            id_map[i] : event_yaml['event'][i]['id']

            if 'name' not in event_yaml['event'][i].keys():
                raise LEDConfigImporter.LEDConfigImporterError(f'No \'name\' in {event_yaml["event"][i]}.')
            name_map[i] : event_yaml['event'][i]['name']

        # Check if names and ID's are unique and do not overlap each other
        if len(id_map.values()) != len(set(id_map.values())):
            raise LEDConfigImporter.LEDConfigImporterError(f'events\' IDs in {file} aren\'t uniqueue.')

        if len(name_map.values()) != len(set(name_map.values())):
            raise LEDConfigImporter.LEDConfigImporterError(f'events\' names in {file} aren\'t uniqueue.')

        if set(id_map.values()) & set(self._imported_animations.keys()):
            raise LEDConfigImporter.LEDConfigImporterError(f'events\' IDs in {file} overlap previous ID declarations.')

        # Create instances of executors for each event
        for event in event_yaml['event']:
            self._imported_animations[(event['id'], event['name'])] = Executor(event, self._num_led, self._time_step, self._global_brightness)

        # Check if imported executors contain all animations required by Panther
        if not set(self._husarion_animations).issubset(self._imported_animations.keys()):
            raise LEDConfigImporter.LEDConfigImporterError(f'no Panther specific animations: {set(self._husarion_animations) - set(self._imported_animations.keys())} in {self._imported_animations.keys()}')


    def get_animation(self, id=None, name=None):
        key_file = self._get_animation_key(self._imported_animations, id=id, name=name)
        key_added = self._get_animation_key(self._added_animations, id=id, name=name)

        if key_file is not None:
            return copy.deepcopy(self._imported_animations[key_file])
        elif key_added is not None:
            return copy.deepcopy(self._added_animations[key_added])
        elif id is not None:
            raise LEDConfigImporter.LEDConfigImporterError(f'animation with ID: {id} is not defined.')
        else:
            raise LEDConfigImporter.LEDConfigImporterError(f'animation with name: {name} is not defined.')


    def _get_animation_key(self, animation_list, id=None, name=None):
        if animation_list:
            if id is not None and name is not None:
                if (id, name) in animation_list.keys():
                    return (id, name)
                else:
                    return None
            if id is not None:
                keys = list(animation_list.keys())
                IDs = (np.array(keys)[:,0]).astype(np.int)
                idx = np.where(IDs == id)[0]

            elif name is not None:
                keys = list(animation_list.keys())
                names = (np.array(keys)[:,1])
                idx = np.where(names == name)[0]
            
            else:
                raise LEDConfigImporter.LEDConfigImporterError(f'ID and name can\'t be none at the same time')

            if len(idx):
                return keys[idx[0]]
        return None


    @property
    def global_brightness(self):
        return self._global_brightness


    @property
    def num_led(self):
        return self._num_led


    @property
    def time_step(self):
        return self._time_step