#!/usr/bin/env python

import os
import sys
import time
import yaml
import random
import unittest
import threading
import numpy as np
from typing import Optional

sys.path.append(os.path.abspath(__file__ + '/../../'))

from src.panther_lights_controller.controller.controller import Controller
from src.panther_lights_controller.led_config_importer import LEDConfigImporter


class DummyController(Controller):

    def __init__(self):
        pass

    def set_panel(self, panel_num, panel_frame, brightness: Optional[int] = 0):
        pass

    def set_panel_state(self, panel_num, state):
        pass

    def set_brightness(self, bright):
        pass

    def clear_panel(self):
        pass


class TestAnimation(unittest.TestCase):

    def setUp(self):
        event_yaml_path = os.path.dirname(os.path.abspath(__file__)) + '/config/events/event_animaton_test.yaml'
        event_yaml_file = open(event_yaml_path,'r')
        self._event_yaml = yaml.load(event_yaml_file, Loader=yaml.Loader)
        event_yaml_file.close()


        conf_path = os.path.dirname(os.path.abspath(__file__)) + '/config/led_conf/animation_test_led_conf.yaml'
        conf_file = open(conf_path,'r')
        conf_yaml = yaml.load(conf_file, Loader=yaml.Loader)
        conf_file.close()
        self._led_config_importer = LEDConfigImporter(conf_yaml)
        self._dummy_controller = DummyController()


    def test_duration_and_consistancy(self):
        for i in range(len(self._event_yaml['event'])):
            if 'both' in self._event_yaml['event'][i]['animation']:
                duration = self._event_yaml['event'][i]['animation']['both']['duration']
                if 'repeat' in self._event_yaml['event'][i]['animation']['both']:
                    repeat = self._event_yaml['event'][i]['animation']['both']['repeat']
                else:
                    repeat = 1
                estiamted_time = duration * repeat
            else:
                duration = self._event_yaml['event'][i]['animation']['front']['duration']
                if 'repeat' in self._event_yaml['event'][i]['animation']['front']:
                    repeat = self._event_yaml['event'][i]['animation']['front']['repeat']
                else:
                    repeat = 1
                estiamted_time_front = duration * repeat

                duration = self._event_yaml['event'][i]['animation']['rear']['duration']
                if 'repeat' in self._event_yaml['event'][i]['animation']['rear']:
                    repeat = self._event_yaml['event'][i]['animation']['rear']['repeat']
                else:
                    repeat = 1
                estiamted_time_rear = duration * repeat
                estiamted_time = max(estiamted_time_front, estiamted_time_rear)
                

            times = np.zeros(10)
            for j in range(len(times)):
                key = self._led_config_importer.event_key(i+1)
                anim = self._led_config_importer.get_event(key)
                anim.spawn(self._dummy_controller)
                start = time.time()
                anim.run()
                while not anim.finished:
                    time.sleep(0.00000001)
                times[j] = time.time() - start
                del anim
            finish_avg = np.mean(times)
            deviation = (abs(finish_avg - estiamted_time) / estiamted_time)
            covariance = np.cov(times)
            self.assertTrue(deviation < 0.1, f'failed for animation {i}. Expected time: {estiamted_time}, ' \
                            + f'Average time: {finish_avg}, Deviation: {deviation}')
            self.assertTrue(covariance < 1e-5, f'failed for animation {i}. Covariance: {covariance}')


    def test_stop_run(self):
        duration = self._event_yaml['event'][0]['animation']['both']['duration']
        repeat = self._event_yaml['event'][0]['animation']['both']['repeat']
        estiamted_time = duration * repeat

        percentages = [0.1, 0.3, 0.69, 0.8, 0.9]
        sleep_between = [0.5, 1, 1.5, 2]

        for s in sleep_between:
            for p in percentages:
                key = self._led_config_importer.event_key(1)
                anim = self._led_config_importer.get_event(key)
                anim.spawn(self._dummy_controller)
                start = time.time()
                anim.run()
                time.sleep(estiamted_time * p)
                anim.stop()
                time.sleep(s)
                anim.run()
                while not anim.finished:
                    time.sleep(0.00000001)
                finish = time.time() - start
                del anim

                deviation = (abs(finish - estiamted_time - s) / estiamted_time)
                self.assertTrue(deviation < 0.15, f'failed for animation {0}. Expected time: {estiamted_time}, ' \
                                + f'Average time: {finish}, Deviation: {deviation}')


    def test_reset(self):
        duration = self._event_yaml['event'][0]['animation']['both']['duration']
        repeat = self._event_yaml['event'][0]['animation']['both']['repeat']
        estiamted_time = duration * repeat

        percentages = [0.1, 0.3, 0.69, 0.8, 0.9]

        for p in percentages:
            key = self._led_config_importer.event_key(1)
            anim = self._led_config_importer.get_event(key)
            anim.spawn(self._dummy_controller)
            start = time.time()
            anim.run()
            time.sleep(estiamted_time * p)
            anim.stop()
            anim.reset()
            anim.run()
            while not anim.finished:
                time.sleep(0.00000001)
            finish = time.time() - start
            del anim

            deviation = (abs(finish - estiamted_time - (estiamted_time * p)) / estiamted_time)
            self.assertTrue(deviation < 0.1, f'failed for animation {0}. Expected time: {estiamted_time}, ' \
                            + f'Average time: {finish}, Deviation: {deviation}')
    

    def test_percentage(self):
        duration = self._event_yaml['event'][0]['animation']['both']['duration']
        repeat = self._event_yaml['event'][0]['animation']['both']['repeat']
        estiamted_time = duration * repeat
        percentages = [0.1, 0.3, 0.69, 0.8, 1]

        for p in percentages:
            key = self._led_config_importer.event_key(1)
            anim = self._led_config_importer.get_event(key)
            anim.spawn(self._dummy_controller)
            start = time.time()
            anim.run()
            time.sleep(estiamted_time * p)
            anim.stop()
            deviation = (abs(p - anim.percent_done) / p)
            self.assertTrue(deviation < 0.01, f'failed for animation {0}. Expected percentage: {p}, ' \
                            + f'Read percentage: {anim.percent_done}, Deviation: {deviation}')
            del anim

        for p in percentages:
            key = self._led_config_importer.event_key(1)
            anim = self._led_config_importer.get_event(key)
            anim.spawn(self._dummy_controller)
            start = time.time()
            anim.run()
            time.sleep(estiamted_time * p / 2)
            anim.stop()
            time.sleep(estiamted_time * p)
            anim.run()
            time.sleep(estiamted_time * p / 2)
            anim.stop()
            deviation = (abs(p - anim.percent_done) / p)
            self.assertTrue(deviation < 0.01, f'failed for animation {0}. Expected percentage: {p}, ' \
                            + f'Read percentage: {anim.percent_done}, Deviation: {deviation}')
            del anim

        for p in percentages:
            key = self._led_config_importer.event_key(1)
            anim = self._led_config_importer.get_event(key)
            anim.spawn(self._dummy_controller)
            start = time.time()
            anim.run()
            time.sleep(estiamted_time * p)
            percentage = anim.percent_done
            anim.stop()
            deviation = (abs(p - percentage) / p)
            self.assertTrue(deviation < 0.01, f'failed for animation {0}. Expected percentage: {p}, ' \
                            + f'Read percentage: {percentage}, Deviation: {deviation}')
            del anim

        for p in percentages:
            key = self._led_config_importer.event_key(1)
            anim = self._led_config_importer.get_event(key)
            anim.spawn(self._dummy_controller)
            start = time.time()
            anim.run()
            time.sleep(estiamted_time * p / 2)
            anim.stop()
            time.sleep(estiamted_time * p)
            anim.run()
            time.sleep(estiamted_time * p / 2)
            percentage = anim.percent_done
            anim.stop()
            deviation = (abs(p - percentage) / p)
            self.assertTrue(deviation < 0.01, f'failed for animation {0}. Expected percentage: {p}, ' \
                            + f'Read percentage: {percentage}, Deviation: {deviation}')
            del anim


    def test_cleaning_threads(self):
        random.seed(time.time())
        anims = []
        base_thread_count = threading.active_count()
        for i in range(20):
            spawn_count = int(random.random() * 15 + 1)
            for _ in range(spawn_count):
                a_num = int(random.random() * 5 + 1)
                key = self._led_config_importer.event_key(a_num)
                anim = self._led_config_importer.get_event(key)
                anim.spawn(self._dummy_controller)
                anims.append(anim)
            self.assertEqual(base_thread_count+spawn_count*2, threading.active_count(), 'threads counts doesn\'t match')

            time.sleep(random.random()/50)
            a_num = None
            while anims:
                if a_num:
                    if a_num < len(anims):
                        anims[a_num].stop()

                a_num = int(random.random() * len(anims))
                if not anims[a_num].finished:
                    if random.random() > 0.2:
                        anims[a_num].run()
                        random.seed(time.time())
                        time.sleep(random.random()/10000)
                    else:
                        anims[a_num].stop()
                else:
                    a = anims.pop(a_num)
                    del a
            self.assertEqual(base_thread_count, threading.active_count(), 'thread count\'s doesn\'t match')


if __name__ == '__main__':
    unittest.main()
