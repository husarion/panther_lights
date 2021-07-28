import os
import yaml
import copy
from typing import Optional

from .animations import *
from .event import Event

class LEDConfigImporter:

    class LEDConfigImporterError(Exception):
        '''basic error for LEDConfigImporter'''
        def __init__(self, message='YAML keyword error'):
            self.message = message
            super().__init__(self.message)


    def __init__(self, yaml_file, global_brightness, num_led):
        '''class importing panel configuration and events definitions'''
        raise LEDConfigImporter.LEDConfigImporterError(f'no')
        self._yaml = yaml_file

        # Check if obligatory keyword exist
        global_keywords = ['event_animations_files']
        if not set(global_keywords).issubset(self._yaml.keys()):
            missing_keywords = set(global_keywords) - set(self._yaml.keys())
            raise LEDConfigImporter.LEDConfigImporterError(f'no {missing_keywords}')

        self._global_brightness = global_brightness
        if not (0 < self._global_brightness <= 1):
            raise LEDConfigImporter.LEDConfigImporterError(f'brightness has match boundaries 0 < brightness <= 1')

        self._global_brightness = int(round(self._global_brightness * 255))
        self._num_led = num_led
        self._imported_animations = {}

        self._id_map = {}
        self._name_map = {}

        for event_file_path in self._yaml['event_animations_files']:
            if not os.path.isabs(event_file_path):
                src_path = os.path.dirname(os.path.abspath(__file__))
                event_file_path = os.path.join(src_path, f'../../config/{event_file_path}')

            event_file = open(event_file_path,'r')
            event_yaml = yaml.load(event_file, Loader=yaml.Loader)
            event_file.close()

            self._import_file(event_yaml)


    def _import_file(self, event_yaml):
        '''imports events from given file'''
        if 'event' not in event_yaml.keys():
            raise LEDConfigImporter.LEDConfigImporterError(f'no \'event\' in {file}')

        # create map of all names in file
        id_map_offset = 0
        if self._id_map:
            id_map_offset = max(self._id_map.keys()) + 1

        for i in range(len(event_yaml['event'])):
            if 'id' not in event_yaml['event'][i].keys():
                raise LEDConfigImporter.LEDConfigImporterError(f'no \'id\' in {event_yaml["event"][i]}')
            self._id_map[i+id_map_offset] = event_yaml['event'][i]['id']

            if 'name' not in event_yaml['event'][i].keys():
                raise LEDConfigImporter.LEDConfigImporterError(f'no \'name\' in {event_yaml["event"][i]}')
            self._name_map[i+id_map_offset] = event_yaml['event'][i]['name']

        # check if names and ID's are unique and do not overlap each other
        if len(self._id_map.values()) != len(set(self._id_map.values())):
            raise LEDConfigImporter.LEDConfigImporterError(
                f'events\' IDs in {event_yaml} aren\'t uniqueue')

        if len(self._name_map.values()) != len(set(self._name_map.values())):
            raise LEDConfigImporter.LEDConfigImporterError(
                f'events\' names in {event_yaml} aren\'t uniqueue')

        if set(self._id_map.values()) & set(self._imported_animations.keys()):
            raise LEDConfigImporter.LEDConfigImporterError(
                f'events\' IDs in {event_yaml} overlap previous ID declarations')

        # create instance of each event
        for event in event_yaml['event']:
            if event['id'] <= 0 or event['name'] == '':
                raise LEDConfigImporter.LEDConfigImporterError(
                    f'event id has to be non negative and name has can\'t be empty string')
            self._imported_animations[(event['id'], event['name'])] = Event(event,
                                                                            self._num_led,
                                                                            self._global_brightness)


    def get_event(self, key):
        '''returns event by it's key, name or both'''
        return copy.deepcopy(self._imported_animations[key])


    def event_key(self, id: Optional[int]=None, name: Optional[int]=None):
        '''checks if given id or name exists in all imported animations and returns it's key'''
        key = self._get_animation_list_key(self._imported_animations, id=id, name=name)
        if key is not None:
            return key
        elif id is not None:
            raise LEDConfigImporter.LEDConfigImporterError(f'animation with ID: {id} is not defined')
        else:
            raise LEDConfigImporter.LEDConfigImporterError(f'animation with name: {name} is not defined')


    def _get_animation_list_key(self, animation_list, id: Optional[int]=None, name: Optional[int]=None):
        '''checks if event with given key or name exists in given animation list and returns it key'''
        if animation_list:
            if id is not None and name is not None:
                if (id, name) in animation_list.keys():
                    return (id, name)
                else:
                    return None
            idx = None
            keys = list(animation_list.keys())
            if id is not None:
                IDs = [key[0] for key in keys]
                for i in range(len(IDs)):
                    if IDs[i] == id:
                        idx = i
                        break

            elif name is not None:
                names = [key[1] for key in keys]
                for i in range(len(names)):
                    if names[i] == name:
                        idx = i
                        break
            
            else:
                raise LEDConfigImporter.LEDConfigImporterError(f'ID and name can\'t be none at the same time')

            if not idx is None:
                return keys[idx]
        return None


    @property
    def global_brightness(self):
        '''returns imported global brightness'''
        return self._global_brightness


    @property
    def num_led(self):
        '''returns imported LED count'''
        return self._num_led
