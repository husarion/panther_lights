import time
import queue
from enum import Enum
from threading import Thread, Lock
from typing import Optional

from apa102_pi.driver import apa102 
import RPi.GPIO as GPIO

from .controller import Controller

class HardwareAPA102Controller(Controller):
    LED_SWITCH_FRONT_STATE = True   # High - front panel
    LED_SWITCH_REAR_STATE  = False  # Low  - rear panel
    LED_POWER_ON_STATE     = False  # active LEDs with low state



    def __init__(self,
                 num_led,
                 panel_count: Optional[int] = 2,
                 brightness: Optional[int] = 15,
                 led_switch_pin: Optional[int] = 20,
                 led_power_pin: Optional[int] = 26):
        '''hardware abstraction of APA102 LEDs used in Panther robot'''

        self._num_led = num_led
        self._panel_count = panel_count
        self._global_brightness = brightness

        self._is_running = True
        self._thread_finished = False
        self._lock = Lock()
        self._anim_lock = Lock()
        self._queue = queue.Queue()
        self._pixels = apa102.APA102(num_led=self._num_led,
                                     order='rgb',
                                     mosi=10,
                                     sclk=11,
                                     global_brightness=brightness)
        self._led_switch_pin = led_switch_pin
        self._led_power_pin = led_power_pin
        self._front_active = True
        self._rear_active = True

        # setup and activate panels
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._led_switch_pin, GPIO.OUT)
        GPIO.setup(self._led_power_pin, GPIO.OUT)
        GPIO.output((self._led_switch_pin, self._led_power_pin),
                    (HardwareAPA102Controller.LED_SWITCH_FRONT_STATE, HardwareAPA102Controller.LED_POWER_ON_STATE))

        self._anim_update_thread = Thread(target=self._update_anim, daemon=True)
        self._anim_update_thread.start()


    def set_panel(self, panel_num, panel_frame, brightness: Optional[int]=None):
        '''sets panel_frame on panel_num led panel'''
        with self._lock:

            with self._anim_lock:
                self._queue.put((panel_num, panel_frame, brightness))

    
    def _update_anim(self):

        panel = (0, [0]*self._num_led, None)

        with self._anim_lock:
            running = self._is_running

        while running:
            try:
                with self._anim_lock:
                    panel = self._queue.get(block=True, timeout=0.000001)
            except queue.Empty:
                pass
            
            if panel[2] is None:
                brightness = self._global_brightness
            else:
                brightness = panel[2]

            # select panel
            if panel[0] == 0 and self._front_active:
                GPIO.output(self._led_switch_pin, HardwareAPA102Controller.LED_SWITCH_FRONT_STATE)
            elif panel[0] == 1 and self._rear_active:
                GPIO.output(self._led_switch_pin, HardwareAPA102Controller.LED_SWITCH_REAR_STATE)
            else:
                raise ValueError('panther lights have only two panels')

            # set all leds in this panel
            for i in range(self._num_led):
                self._pixels.set_pixel_rgb(i, int(panel[1][i]), brightness)
            self._pixels.show()

            with self._anim_lock:
                running = self._is_running

        self._pixels.cleanup()
        GPIO.output(self._led_power_pin, not HardwareAPA102Controller.LED_POWER_ON_STATE)
        GPIO.cleanup()

        with self._anim_lock:
            self._thread_finished = True


    def set_panel_state(self, panel_num, state):
        '''locks or unlocks given panel'''
        if not state:
            self.clear_panel(panel_num)
        with self._lock:
            if panel_num == 0:
                self._front_active = state
            elif panel_num == 1:
                self._rear_active = state

    
    def set_brightness(self, bright):
        '''sets panel brightness'''
        with self._lock:
            self._global_brightness = brightness


    def clear_panel(self, panel_num):
        '''clears panel'''
        self.set_panel(panel_num, [0]*self._num_led)

    
    def __del__(self):
        '''cleans all leds and turns off both panels'''
        with self._anim_lock:
            self._is_running = False
        finished = False
        while not finished:
            with self._anim_lock:
                finished = self._thread_finished
            time.sleep(0.00001)
        self._anim_update_thread.join()
