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


    def __init__(self, yaml_file):
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

        self._yaml = yaml_file

        # Check if obligatory keyword exist
        global_keywords = ['global_brightness', 'num_led', 'time_step', 'event_animations_files']
        if not set(global_keywords).issubset(self._yaml.keys()):
            raise LEDConfigImporter.LEDConfigImporterError(f'No {set(global_keywords) - set(self._yaml.keys())}')

        self._global_brightness = float(self._yaml['global_brightness'])
        if not (0 < self._global_brightness <= 1):
            raise LEDConfigImporter.LEDConfigImporterError('brightness has match boundaries 0 < brightness <= 1.')
        self._global_brightness = int(round(self._global_brightness * 255))

        self._num_led = self._yaml['num_led']
        self._time_step = self._yaml['time_step']
        self._imported_animations = {}
        self._added_animations = {}

        self._id_map = {}
        self._name_map = {}


        for event_file_path in self._yaml['event_animations_files']:
            if not os.path.isabs(event_file_path):
                src_path = os.path.dirname(os.path.abspath(__file__))
                event_file_path = os.path.join(src_path, f'../../config/{file}')

            event_file = open(event_file_path,'r')
            event_yaml = yaml.load(event_file, Loader=yaml.Loader)
            event_file.close()

            self._import_file(event_yaml)

    

    def _import_file(self, event_yaml):
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

        if 'event' not in event_yaml.keys():
            raise LEDConfigImporter.LEDConfigImporterError(f'No \'event\' in {file}.')


        # Create map of all names in file
        id_map_offset = 0
        if self._id_map:
            id_map_offset = max(self._id_map.keys()) + 1

        for i in range(len(event_yaml['event'])):
            if 'id' not in event_yaml['event'][i].keys():
                raise LEDConfigImporter.LEDConfigImporterError(f'No \'id\' in {event_yaml["event"][i]}.')
            self._id_map[i+id_map_offset] = event_yaml['event'][i]['id']

            if 'name' not in event_yaml['event'][i].keys():
                raise LEDConfigImporter.LEDConfigImporterError(f'No \'name\' in {event_yaml["event"][i]}.')
            self._name_map[i+id_map_offset] = event_yaml['event'][i]['name']

        # Check if names and ID's are unique and do not overlap each other
        if len(self._id_map.values()) != len(set(self._id_map.values())):
            raise LEDConfigImporter.LEDConfigImporterError(f'events\' IDs in {event_yaml} aren\'t uniqueue.')

        if len(self._name_map.values()) != len(set(self._name_map.values())):
            raise LEDConfigImporter.LEDConfigImporterError(f'events\' names in {event_yaml} aren\'t uniqueue.')

        if set(self._id_map.values()) & set(self._imported_animations.keys()):
            raise LEDConfigImporter.LEDConfigImporterError(f'events\' IDs in {event_yaml} overlap previous ID declarations.')

        # Create instances of executors for each event
        for event in event_yaml['event']:
            if event['id'] <= 0 or event['name'] == '':
                raise LEDConfigImporter.LEDConfigImporterError(f'event id has to be non negative and name has can\'t be empty string.')
            self._imported_animations[(event['id'], event['name'])] = Executor(event, self._num_led, self._time_step, self._global_brightness)


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