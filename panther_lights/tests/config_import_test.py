#!/usr/bin/env python

import sys
import os
sys.path.append(os.path.abspath(__file__ + '/../../'))

import unittest
import yaml

from src.panther_lights.executor import Executor
from src.panther_lights.animations.image_animation import ImageAnimation


class TestImport(unittest.TestCase):

    def test_yaml_import(self):
        conf_path = os.path.dirname(os.path.abspath(__file__)) + '/config/events_single_both.yaml'
        conf_file = open(conf_path,'r')
        conf_yaml = yaml.load(conf_file, Loader=yaml.Loader)
        conf_file.close()
        img = conf_yaml['event'][0]['animation']['both']['image']
        conf_yaml['event'][0]['animation']['both']['image'] = os.path.dirname(os.path.abspath(__file__)) + '/animations/' + img
        ex = Executor(conf_yaml['event'][0], 48, 0.01, 255)





class TestBothInterrupting(unittest.TestCase):

    def setUp(self):
        conf_path = os.path.dirname(os.path.abspath(__file__)) + '/config/events_single_both.yaml'
        conf_file = open(conf_path,'r')
        conf_yaml = yaml.load(conf_file, Loader=yaml.Loader)
        conf_file.close()
        img = conf_yaml['event'][0]['animation']['both']['image']
        conf_yaml['event'][0]['animation']['both']['image'] = os.path.dirname(os.path.abspath(__file__)) + '/animations/' + img
        self._ex = Executor(conf_yaml['event'][0], 48, 0.01, 255)

    def test_id(self):
        self.assertEqual(self._ex._id, 10, 'incorrect id')

    def test_interrupting(self):
        self.assertEqual(self._ex._interrupting, False, 'incorrect interrupting flag')





class TestBothDefaultParameters(unittest.TestCase):

    def setUp(self):
        conf_path = os.path.dirname(os.path.abspath(__file__)) + '/config/events_single_both.yaml'
        conf_file = open(conf_path,'r')
        conf_yaml = yaml.load(conf_file, Loader=yaml.Loader)
        conf_file.close()
        img = conf_yaml['event'][0]['animation']['both']['image']
        conf_yaml['event'][0]['animation']['both']['image'] = os.path.dirname(os.path.abspath(__file__)) + '/animations/' + img
        self._ex = Executor(conf_yaml['event'][0], 48, 0.01, 255)

    def test_assert_animaition_type(self):
        self.assertTrue(isinstance(self._ex._front_animation, ImageAnimation), 'front animation type is not matching ImageAnimation')
        self.assertTrue(isinstance(self._ex._tail_animation,  ImageAnimation), 'tail animation type is not matching ImageAnimation')

    def test_assert_parameters(self):
        self.assertEqual(self._ex._front_animation._num_led,    48,    'incorrect imported num_leds')
        self.assertEqual(self._ex._front_animation._time_step,  0.01,  'incorrect imported time_step')
        self.assertEqual(self._ex._front_animation._brightness, 255,   'incorrect default brightness')

        self.assertEqual(self._ex._front_animation._duration,   1,     'incorrect imported duration')
        self.assertEqual(self._ex._front_animation._loops,      1,     'incorrect default loops')
        self.assertEqual(self._ex._front_animation._keep_state, False, 'incorrect default keep state')

    def test_assert_equal_front_tail(self):
        self.assertEqual(self._ex._front_animation._duration,   self._ex._tail_animation._duration,   'duration doesn\'t match front and tail')
        self.assertEqual(self._ex._front_animation._loops,      self._ex._tail_animation._loops,      'loops doen\'t match front and tail')
        self.assertEqual(self._ex._front_animation._keep_state, self._ex._tail_animation._keep_state, 'keep_state doesn\'t match front and tail')





class TestBothNonDefaultParameters(unittest.TestCase):

    def setUp(self):
        conf_path = os.path.dirname(os.path.abspath(__file__)) + '/config/events_single_both_non_default.yaml'
        conf_file = open(conf_path,'r')
        conf_yaml = yaml.load(conf_file, Loader=yaml.Loader)
        conf_file.close()
        img = conf_yaml['event'][0]['animation']['both']['image']
        conf_yaml['event'][0]['animation']['both']['image'] = os.path.dirname(os.path.abspath(__file__)) + '/animations/' + img
        self._ex = Executor(conf_yaml['event'][0], 48, 0.01, 255)

    def test_assert_animaition_type(self):
        self.assertTrue(isinstance(self._ex._front_animation, ImageAnimation), 'front animation type is not matching ImageAnimation')
        self.assertTrue(isinstance(self._ex._tail_animation,  ImageAnimation), 'tail animation type is not matching ImageAnimation')

    def test_assert_parameters(self):
        self.assertEqual(self._ex._front_animation._duration,   2,    'incorrect imported duration')
        self.assertEqual(self._ex._front_animation._loops,      3,    'incorrect imported loops')
        self.assertEqual(self._ex._front_animation._keep_state, True, 'incorrect imported keep state')





class TestDifferentForntTail(unittest.TestCase):

    def setUp(self):
        conf_path = os.path.dirname(os.path.abspath(__file__)) + '/config/events_single_different_fornt_tail.yaml'
        conf_file = open(conf_path,'r')
        conf_yaml = yaml.load(conf_file, Loader=yaml.Loader)
        conf_file.close()
        img = conf_yaml['event'][0]['animation']['front']['image']
        conf_yaml['event'][0]['animation']['front']['image'] = os.path.dirname(os.path.abspath(__file__)) + '/animations/' + img

        img = conf_yaml['event'][0]['animation']['tail']['image']
        conf_yaml['event'][0]['animation']['tail']['image'] = os.path.dirname(os.path.abspath(__file__)) + '/animations/' + img
        self._ex = Executor(conf_yaml['event'][0], 48, 0.01, 255)

    def test_assert_animaition_type(self):
        self.assertTrue(isinstance(self._ex._front_animation, ImageAnimation), 'front animation type is not matching ImageAnimation')
        self.assertTrue(isinstance(self._ex._tail_animation,  ImageAnimation), 'tail animation type is not matching ImageAnimation')

    def test_assert_parameters(self):
        self.assertEqual(self._ex._front_animation._duration,   4,     'Incorrect front imported duration')
        self.assertEqual(self._ex._front_animation._loops,      3,     'Incorrect front imported loops')
        self.assertEqual(self._ex._front_animation._keep_state, True,  'Incorrect front imported keep state')

        self.assertEqual(self._ex._tail_animation._duration,    5,     'Incorrect tail imported duration')
        self.assertEqual(self._ex._tail_animation._loops,       4,     'Incorrect tail imported loops')
        self.assertEqual(self._ex._tail_animation._keep_state,  False, 'Incorrect tail imported keep state')





class TestMultipleAnimations(unittest.TestCase):

    def setUp(self):
        conf_path = os.path.dirname(os.path.abspath(__file__)) + '/config/events_multiple_animations.yaml'
        conf_file = open(conf_path,'r')
        conf_yaml = yaml.load(conf_file, Loader=yaml.Loader)
        conf_file.close()
        img = conf_yaml['event'][0]['animation']['both']['image']
        conf_yaml['event'][0]['animation']['both']['image'] = os.path.dirname(os.path.abspath(__file__)) + '/animations/' + img

        img = conf_yaml['event'][1]['animation']['both']['image']
        conf_yaml['event'][1]['animation']['both']['image'] = os.path.dirname(os.path.abspath(__file__)) + '/animations/' + img
        self._ex_0 = Executor(conf_yaml['event'][0], 48, 0.01, 255)
        self._ex_1 = Executor(conf_yaml['event'][1], 48, 0.01, 255)

    def test_id(self):
        self.assertEqual(self._ex_0._id, 1, 'incorrect ex_0 id')
        self.assertEqual(self._ex_1._id, 2, 'incorrect ex_1 id')





if __name__ == '__main__':
    unittest.main()
