import queue
import time
import os

from driver import VirtualLEDController, HardwareAPA102Controller
from executor import PantherLightsAnimationExecutorThread, PantherLightsQueueElem
from animation import LEDConfigImporter

class PantherLights:
    def __init__(self,
                 config_path=None,
                 use_virtual_driver=False):

        if config_path is None:
            config_path = '../../config/led_conf.yaml'

        src_path = os.path.dirname(__file__)
        conf_path = os.path.relpath(config_path, src_path)

        self._led_config_importer = LEDConfigImporter(conf_path)

        self._time_step = self._led_config_importer.time_step
        self._num_led = self._led_config_importer.num_led
        self._global_brightness = self._led_config_importer.time_step

        self._driver = None

        if use_virtual_driver:
            try:
                import matplotlib
            except ImportError:
                raise ImportError('Unable to import matplotlib. Make sure you have installed matplotlib.')
            self._driver = VirtualLEDController(num_led=self._num_led)

        else:
            try:
                from apa102_pi.driver import apa102 
                import RPi.GPIO as GPIO
            except ImportError:
                raise ImportError('No hardware specific packages installed. Make sure you are running this node on Raspberry pi.')
            self._driver = HardwareAPA102Controller(num_led=self._num_led,
                                                    led_switch_pin=self._led_config_importer.led_switch_pin,
                                                    led_power_pin=self._led_config_importer.led_power_pin,
                                                    brightness=self._global_brightness)


        self._queue = queue.Queue()
        self._exector = PantherLightsAnimationExecutorThread(self._queue, self._driver, self._time_step)


    def start(self):
        self._exector.start()
        self._queue.put(PantherLightsQueueElem(self._led_config_importer.get_animation_by_id(10), is_background=True))
        time.sleep(3)
        self._queue.put(PantherLightsQueueElem(self._led_config_importer.get_animation_by_id(25)))
        time.sleep(0.5)
        self._queue.put(PantherLightsQueueElem(self._led_config_importer.get_animation_by_id(20), is_background=True))
        time.sleep(10)
        self._queue.put(PantherLightsQueueElem(self._led_config_importer.get_animation_by_name('TURN-LEFT'), interrupt=True))
        time.sleep(5)
        self._exector.join()
        del self._driver


if __name__ == '__main__':
    pl = PantherLights(use_virtual_driver=True)
    pl.start()