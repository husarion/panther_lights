#!/usr/bin/env python

import sys
import os
sys.path.append(os.path.abspath(__file__ + '/../../'))

import unittest
import yaml

from src.panther_lights.event import Event
from src.panther_lights.animations.animation import Animation
from src.panther_lights.led_config_importer import LEDConfigImporter
from src.panther_lights.animations.image_animation import ImageAnimation
from src.panther_lights.animations.battery_animation import BatteryAnimation


class TestImport(unittest.TestCase):

    def test_yaml_import(self):
        try:
            conf_path = os.path.dirname(os.path.abspath(__file__)) + '/config/events/events_single_both.yaml'
            conf_file = open(conf_path,'r')
            conf_yaml = yaml.load(conf_file, Loader=yaml.Loader)
            conf_file.close()
            Event(conf_yaml['event'][0], 46, 255)
        except Exception as e:
            self.fail(f'Failed to create Event with correct data: {e}')



class TestYamlImporter(unittest.TestCase):

    def setUp(self):
        conf_path = os.path.dirname(os.path.abspath(__file__)) + '/config/events/events_single_both_non_default.yaml'
        conf_file = open(conf_path,'r')
        conf_yaml = yaml.load(conf_file, Loader=yaml.Loader)
        conf_file.close()
        self._event = conf_yaml['event'][0]


    def test_duration(self):
        try:
            for duration in [0.1, 1, 2, 3, 3.5]:
                self._event['animation']['both']['duration'] = duration
                Event(self._event, 46, 255)
        except Exception as e:
            self.fail(f'Failed to create Event with correct data: {e}')

        for duration in [0, -1, -2, -0.1]:
            self._event['animation']['both']['duration'] = duration
            self.assertRaises(Animation.AnimationYAMLError, Event, self._event, 46, 255)

        del self._event['animation']['both']['duration']
        self.assertRaises(Animation.AnimationYAMLError, Event, self._event, 46, 255)


    def test_repeat(self):
        for repeat in [0, -1, -2, -0.1, 0.1, 2.1, 2.0, 0.0]:
            self._event['animation']['both']['repeat'] = repeat
            self.assertRaises(Animation.AnimationYAMLError, Event, self._event, 46, 255)
        try:
            for repeat in [1, 2, 3, 200]:
                self._event['animation']['both']['repeat'] = repeat
                Event(self._event, 46, 255)

            del self._event['animation']['both']['repeat']
            Event(self._event, 46, 255)
        except Exception as e:
            self.fail(f'Failed to create Event with correct data: {e}')


    def test_keep_state(self):
        for keep_state in [-1, 2, 1.0, 0.0]:
            self._event['animation']['both']['keep_state'] = keep_state
            self.assertRaises(Animation.AnimationYAMLError, Event, self._event, 46, 255)

        try:
            for keep_state in [True, False, 1, 0]:
                self._event['animation']['both']['keep_state'] = keep_state
                Event(self._event, 46, 255)

            del self._event['animation']['both']['keep_state']
            Event(self._event, 46, 255)
        except Exception as e:
            self.fail(f'Failed to create Event with correct data: {e}')


    def test_brightness(self):
        for brightness in [-1, 2, 1.1, 0.0, 0]:
            self._event['animation']['both']['brightness'] = brightness
            self.assertRaises(Animation.AnimationYAMLError, Event, self._event, 46, 255)

        try:
            for brightness in [0.1, 0.5, 1, 1.0]:
                self._event['animation']['both']['brightness'] = brightness
                Event(self._event, 46, 255)

            del self._event['animation']['both']['brightness']
            Event(self._event, 46, 255)
        except Exception as e:
            self.fail(f'Failed to create Event with correct data: {e}')





class TestBothDefaultParameters(unittest.TestCase):

    def setUp(self):
        conf_path = os.path.dirname(os.path.abspath(__file__)) + '/config/events/events_single_both.yaml'
        conf_file = open(conf_path,'r')
        conf_yaml = yaml.load(conf_file, Loader=yaml.Loader)
        conf_file.close()
        self._ex = Event(conf_yaml['event'][0], 46, 255)


    def test_assert_animaition_type(self):
        self.assertTrue(isinstance(self._ex._front_animation, ImageAnimation),
                                   'front animation type is not matching ImageAnimation')
        self.assertTrue(isinstance(self._ex._tail_animation, ImageAnimation),
                                   'tail animation type is not matching ImageAnimation')


    def test_assert_parameters(self):
        self.assertEqual(self._ex._front_animation._num_led, 46, 'incorrect imported num_leds')
        self.assertEqual(self._ex._front_animation._brightness, 255, 'incorrect default brightness')

        self.assertEqual(self._ex._front_animation._duration, 1, 'incorrect imported duration')
        self.assertEqual(self._ex._front_animation._loops, 1, 'incorrect default loops')
        self.assertEqual(self._ex._front_animation._keep_state, False, 'incorrect default keep state')


    def test_assert_equal_front_tail(self):
        self.assertEqual(self._ex._front_animation._duration, self._ex._tail_animation._duration,
                         'duration doesn\'t match front and tail')
        self.assertEqual(self._ex._front_animation._loops, self._ex._tail_animation._loops,
                         'loops doen\'t match front and tail')
        self.assertEqual(self._ex._front_animation._keep_state, self._ex._tail_animation._keep_state,
                         'keep_state doesn\'t match front and tail')
        self.assertEqual(self._ex._front_animation._brightness, self._ex._tail_animation._brightness,
                         'brightness doesn\'t match front and tail')



class TestBothNonDefaultParameters(unittest.TestCase):

    def setUp(self):
        conf_path = os.path.dirname(os.path.abspath(__file__)) + '/config/events/events_single_both_non_default.yaml'
        conf_file = open(conf_path,'r')
        conf_yaml = yaml.load(conf_file, Loader=yaml.Loader)
        conf_file.close()

        self._ex = Event(conf_yaml['event'][0], 46, 255)


    def test_assert_animaition_type(self):
        self.assertTrue(isinstance(self._ex._front_animation, ImageAnimation),
                        'front animation type is not matching ImageAnimation')
        self.assertTrue(isinstance(self._ex._tail_animation, ImageAnimation),
                        'tail animation type is not matching ImageAnimation')


    def test_assert_parameters(self):
        self.assertEqual(self._ex._front_animation._duration, 2, 'incorrect imported duration')
        self.assertEqual(self._ex._front_animation._loops, 3, 'incorrect imported loops')
        self.assertEqual(self._ex._front_animation._keep_state, True, 'incorrect imported keep state')
        self.assertEqual(self._ex._front_animation._brightness, 200 , 'incorrect imported brightness')




class TestMultipleAnimations(unittest.TestCase):

    def setUp(self):
        conf_path = os.path.dirname(os.path.abspath(__file__)) + '/config/events/events_multiple_animations.yaml'
        conf_file = open(conf_path,'r')
        conf_yaml = yaml.load(conf_file, Loader=yaml.Loader)
        conf_file.close()

        self._ex_0 = Event(conf_yaml['event'][0], 46, 255)
        self._ex_1 = Event(conf_yaml['event'][1], 46, 255)
        self._ex_2 = Event(conf_yaml['event'][2], 46, 255)


    def test_interrupting(self):
        self.assertEqual(self._ex_0.interrupting, True, 'incorrect interrupting flag')
        self.assertEqual(self._ex_1.interrupting, False, 'incorrect interrupting flag')
        self.assertEqual(self._ex_2.interrupting, False, 'incorrect interrupting flag')


    def test_id(self):
        self.assertEqual(self._ex_0.id, 1, 'incorrect ex_0 id')
        self.assertEqual(self._ex_1.id, 2, 'incorrect ex_1 id')
        self.assertEqual(self._ex_2.id, 3, 'incorrect ex_2 id')




class TestDifferentForntTail(unittest.TestCase):

    def setUp(self):
        conf_path = os.path.dirname(os.path.abspath(__file__)) + \
                    '/config/events/events_single_different_fornt_tail.yaml'
        conf_file = open(conf_path,'r')
        conf_yaml = yaml.load(conf_file, Loader=yaml.Loader)
        conf_file.close()
        self._ex = Event(conf_yaml['event'][0], 46, 255)


    def test_assert_animaition_type(self):
        self.assertTrue(isinstance(self._ex._front_animation, ImageAnimation),
                                   'front animation type is not matching ImageAnimation')
        self.assertTrue(isinstance(self._ex._tail_animation, ImageAnimation),
                                   'tail animation type is not matching ImageAnimation')


    def test_assert_parameters(self):
        self.assertEqual(self._ex._front_animation._duration, 4, 'incorrect front imported duration')
        self.assertEqual(self._ex._front_animation._loops, 3, 'incorrect front imported loops')
        self.assertEqual(self._ex._front_animation._keep_state, True, 'incorrect front imported keep state')
        self.assertEqual(self._ex._front_animation._brightness, 128, 'incorrect front imported brightness')

        self.assertEqual(self._ex._tail_animation._duration, 5, 'incorrect tail imported duration')
        self.assertEqual(self._ex._tail_animation._loops, 4, 'incorrect tail imported loops')
        self.assertEqual(self._ex._tail_animation._keep_state, False, 'incorrect tail imported keep state')
        self.assertEqual(self._ex._tail_animation._brightness, 153, 'incorrect tail imported brightness')



class TestNonImageAniumation(unittest.TestCase):

    def setUp(self):
        conf_path = os.path.dirname(os.path.abspath(__file__)) + \
                    '/config/events/events_single_diffeternt_type_front_tail.yaml'
        conf_file = open(conf_path,'r')
        conf_yaml = yaml.load(conf_file, Loader=yaml.Loader)
        conf_file.close()

        self._ex = Event(conf_yaml['event'][0], 46, 255)


    def test_assert_animaition_type(self):
        self.assertTrue(isinstance(self._ex._front_animation, ImageAnimation),
                        'front animation type is not matching ImageAnimation')
        self.assertTrue(isinstance(self._ex._tail_animation, BatteryAnimation),
                        'tail animation type is not matching ImageAnimation')


class TestLedConfigImportSingleEventsFile(unittest.TestCase):

    def setUp(self):
        conf_path = os.path.dirname(os.path.abspath(__file__)) + '/config/led_conf/single_file_conf.yaml'
        conf_file = open(conf_path,'r')
        self._conf_yaml = yaml.load(conf_file, Loader=yaml.Loader)
        conf_file.close()


    def test_yaml_import(self):
        try:
            LEDConfigImporter(self._conf_yaml)
        except Exception as e:
            self.fail(f'Failed to create LEDConfigImporter with correct data: {e}')


    def test_global_brightness(self):
        try:
            for brightness in [0.1, 0.5, 1, 1.0]:
                self._conf_yaml['global_brightness'] = brightness
                LEDConfigImporter(self._conf_yaml)

        except Exception as e:
            self.fail(f'Failed to create LEDConfigImporter with correct data: {e}')

        for brightness in [-1, 2, 1.1, 0.0, 0]:
            self._conf_yaml['global_brightness'] = brightness
            self.assertRaises(LEDConfigImporter.LEDConfigImporterError, LEDConfigImporter, self._conf_yaml)
        del self._conf_yaml['global_brightness']
        self.assertRaises(LEDConfigImporter.LEDConfigImporterError, LEDConfigImporter, self._conf_yaml)


    def test_num_led(self):
        del self._conf_yaml['num_led']
        self.assertRaises(LEDConfigImporter.LEDConfigImporterError, LEDConfigImporter, self._conf_yaml)


    def test_event_animations_files(self):
        del self._conf_yaml['event_animations_files']
        self.assertRaises(LEDConfigImporter.LEDConfigImporterError, LEDConfigImporter, self._conf_yaml)


    def test_uniqueue_names(self):
        led_conf = None
        self._conf_yaml['event_animations_files'][0] = os.path.dirname(os.path.abspath(__file__)) + \
                        '/config/events/events_mulitple_battery.yaml'
        try:
            led_conf = LEDConfigImporter(self._conf_yaml)
        except Exception as e:
            self.fail(f'Failed to create LEDConfigImporter with correct data: {e}')

        events_conf_yaml = self._import_events()
        events_conf_yaml['event'][0]['id'] = 2
        self.assertRaises(LEDConfigImporter.LEDConfigImporterError, led_conf._import_file, events_conf_yaml)

        events_conf_yaml = self._import_events()
        events_conf_yaml['event'][2]['id'] = 1
        self.assertRaises(LEDConfigImporter.LEDConfigImporterError, led_conf._import_file, events_conf_yaml)
        
        events_conf_yaml = self._import_events()
        events_conf_yaml['event'][2]['name'] = 'TEST_1'
        self.assertRaises(LEDConfigImporter.LEDConfigImporterError, led_conf._import_file, events_conf_yaml)

        events_conf_yaml = self._import_events()
        events_conf_yaml['event'][2]['name'] = 'TEST_2'
        self.assertRaises(LEDConfigImporter.LEDConfigImporterError, led_conf._import_file, events_conf_yaml)

        events_conf_yaml = self._import_events()
        events_conf_yaml['event'][0]['name'] = 'TEST_2'
        events_conf_yaml['event'][2]['id'] = 1
        self.assertRaises(LEDConfigImporter.LEDConfigImporterError, led_conf._import_file, events_conf_yaml)

        events_conf_yaml = self._import_events()
        events_conf_yaml['event'][0]['name'] = ''
        self.assertRaises(LEDConfigImporter.LEDConfigImporterError, led_conf._import_file, events_conf_yaml)

        events_conf_yaml = self._import_events()
        events_conf_yaml['event'][2]['id'] = 0
        self.assertRaises(LEDConfigImporter.LEDConfigImporterError, led_conf._import_file, events_conf_yaml)

        events_conf_yaml = self._import_events()
        events_conf_yaml['event'][2]['id'] = -1
        self.assertRaises(LEDConfigImporter.LEDConfigImporterError, led_conf._import_file, events_conf_yaml)


    def _import_events(self):
        conf_file = open(self._conf_yaml['event_animations_files'][0],'r')
        events_conf_yaml = yaml.load(conf_file, Loader=yaml.Loader)
        conf_file.close()

        return events_conf_yaml


    def test_get_event(self):
        self._conf_yaml['event_animations_files'][0] = os.path.dirname(os.path.abspath(__file__)) + \
                        '/config/events/events_mulitple_battery.yaml'
        led_conf_importer = LEDConfigImporter(self._conf_yaml)

        animations = {1 : 'TEST_1',
                      2 : 'TEST_2',
                      3 : 'TEST_3'}

        for id in animations.keys():
            name = animations[id]
            self.assertEqual(led_conf_importer.get_event(id=id).id, id, 'incorrect animation id')
            self.assertEqual(led_conf_importer.get_event(name=name).id, id, 'incorrect animation id')
            self.assertEqual(led_conf_importer.get_event(id=id,name=name).id, id, 'incorrect animation id')

            self.assertRaises(LEDConfigImporter.LEDConfigImporterError, led_conf_importer.get_event, 0, name)
            self.assertRaises(LEDConfigImporter.LEDConfigImporterError, led_conf_importer.get_event, id, 'TEST_10')

        for id in [-2, -1, 0, 10]:
            self.assertRaises(LEDConfigImporter.LEDConfigImporterError, led_conf_importer.get_event, id=id)

        for name in ['', 'TEST_5']:
            self.assertRaises(LEDConfigImporter.LEDConfigImporterError, led_conf_importer.get_event, name=name)

        




class TestLedConfigImportTwoEventsFiles(unittest.TestCase):

    def setUp(self):
        conf_path = os.path.dirname(os.path.abspath(__file__)) + '/config/led_conf/two_events_file_conf.yaml'
        conf_file = open(conf_path,'r')
        self._conf_yaml = yaml.load(conf_file, Loader=yaml.Loader)
        conf_file.close()


    def test_yaml_import(self):
        led_conf_importer = None
        try:
            led_conf_importer = LEDConfigImporter(self._conf_yaml)
        except Exception as e:
            self.fail(f'Failed to create LEDConfigImporter with correct data: {e}')

        animations = {1 : 'TEST_1',
                      2 : 'TEST_2'}

        for id in animations.keys():
            name = animations[id]
            self.assertEqual(led_conf_importer.get_event(id=id).id, id, 'incorrect animation id')
            self.assertEqual(led_conf_importer.get_event(name=name).id, id, 'incorrect animation id')


    def test_same_file(self):
        self._conf_yaml['event_animations_files'][1] = self._conf_yaml['event_animations_files'][0]
        self.assertRaises(LEDConfigImporter.LEDConfigImporterError, LEDConfigImporter, self._conf_yaml)


    def test_uniqueue_names(self):
        led_conf = LEDConfigImporter(self._conf_yaml)

        events_conf_yaml = self._import_events()
        events_conf_yaml['event'][0]['id'] = 2
        self.assertRaises(LEDConfigImporter.LEDConfigImporterError, led_conf._import_file, events_conf_yaml)

        events_conf_yaml = self._import_events()
        events_conf_yaml['event'][0]['id'] = 1
        self.assertRaises(LEDConfigImporter.LEDConfigImporterError, led_conf._import_file, events_conf_yaml)

        events_conf_yaml = self._import_events()
        events_conf_yaml['event'][0]['name'] = 'TEST_2'
        self.assertRaises(LEDConfigImporter.LEDConfigImporterError, led_conf._import_file, events_conf_yaml)

        events_conf_yaml = self._import_events()
        events_conf_yaml['event'][0]['name'] = 'TEST_1'
        self.assertRaises(LEDConfigImporter.LEDConfigImporterError, led_conf._import_file, events_conf_yaml)


    def _import_events(self):
        conf_file = open(self._conf_yaml['event_animations_files'][0],'r')
        events_conf_yaml = yaml.load(conf_file, Loader=yaml.Loader)
        conf_file.close()

        return events_conf_yaml


if __name__ == '__main__':
    unittest.main()
