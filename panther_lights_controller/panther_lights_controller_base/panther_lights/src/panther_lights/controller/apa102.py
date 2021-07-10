from typing import Optional
from enum import Enum

from apa102_pi.driver import apa102 
import RPi.GPIO as GPIO

from .controller import Controller


class HardwareAPA102Controller(Controller):
    LED_SWITCH_FRONT_STATE = True   # High - front panel
    LED_SWITCH_RESR_STATE  = False  # Low  - rear panel
    LED_POWER_ON_STATE     = False  # active LEDs with low state 
    GLOBAL_MAX_BRIGHTNESS  = 15 



    def __init__(self,
                 num_led,
                 panel_count: Optional[int] = 2,
                 brightness: Optional[int] = HardwareAPA102Controller.GLOBAL_MAX_BRIGHTNESS,
                 led_switch_pin: Optional[int] = 20,
                 led_power_pin: Optional[int] = 26):
        '''hardware abstraction of APA102 LEDs used in Panther robot'''

        self._num_led = num_led
        self._panel_count = panel_count
        self._global_brightness = brightness

        self._lock = Lock()
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


    def set_panel(self, panel_num, panel_frame, brightness: Optional[int]=None):
        '''sets panel_frame on panel_num led panel'''
        with self._lock:
            if brightness is None:
                brightness = self._global_brightness

            # select panel
            if panel_num == 0 and self._front_active:
                GPIO.output(self._led_switch_pin, HardwareAPA102Controller.LED_SWITCH_FRONT_STATE)
            elif panel_num == 1 and self._rear_active:
                GPIO.output(self._led_switch_pin, HardwareAPA102Controller.LED_SWITCH_REAR_STATE)
            else:
                raise ValueError('panther lights have only two panels')

        # set all leds in this panel
        for i in range(self._num_led):
            self._pixels.set_pixel_rgb(i, panel_frame[i], brightness)
        self._pixels.show()


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
        self._pixels.cleanup()
        GPIO.output(self._led_power_pin, not HardwareAPA102Controller.LED_POWER_ON_STATE)
        GPIO.cleanup()