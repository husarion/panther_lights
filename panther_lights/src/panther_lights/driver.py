from threading import Thread
from queue import Queue
import numpy as np
import time

from typing import Optional
from enum import Enum


try:
    import matplotlib.pyplot as plt
    import warnings
    warnings.filterwarnings("ignore")
except ImportError:
    pass


try:
    from apa102_pi.driver import apa102 
    import RPi.GPIO as GPIO
except ImportError:
    pass


class ControllerInterface():
    '''
    LED controller interface
    '''
    def __init__(self, num_led, panel_count, brightness):
        raise NotImplementedError

    def set_panel(self, panel_num, panel_frame, brightness):
        raise NotImplementedError


class VirtualLEDController(ControllerInterface):
    def __init__(self, 
                 num_led,
                 panel_count: Optional[int] = 2,
                 brightness: Optional[int] = 255):

        self._num_led = num_led
        self._panel_count = panel_count
        self._global_brightness = brightness
        self._frame = np.zeros((panel_count, num_led, 3))
        self._panel_states = np.ones(panel_count).astype(np.bool)

        self._queue = Queue()
        self._is_running = True
        self._anim_update_thread = Thread(target=self._update_anim, daemon=True)
        self._anim_update_thread.start()



    def set_panel(self, panel_num, panel_frame, brightness: Optional[int] = None):
        '''
        panel_num (int): panel number.
        panel_frame (Array): new frame to set. Dimensions: (3, led count in single panel)
        '''
        assert 0 <= panel_num < self._panel_count
        assert np.shape(panel_frame)[0] == self._num_led and np.shape(panel_frame)[1] == 3
        panel_frame = panel_frame.T

        if self._panel_states[panel_num]:
            if brightness is None:
                brightness = self._global_brightness

            panel_frame = np.array(panel_frame) * (brightness / 255)

            self._frame[panel_num,:,0] = panel_frame[0]
            self._frame[panel_num,:,1] = panel_frame[1]
            self._frame[panel_num,:,2] = panel_frame[2]

            self._queue.put(self._frame)

    def _update_anim(self):

        plt.ion()
        self._im = plt.imshow(self._frame.astype(np.uint8), vmin=0, vmax=255, animated=True, interpolation='nearest')
        plt.xticks([]), plt.yticks([])

        while self._is_running:
            panel_frame = self._queue.get()
            
            self._im.set_array(panel_frame.astype(np.uint8))
            plt.pause(0.001)


    def set_panel_state(self, panel_num, state):
        assert 0 <= panel < self._panel_counts
        self._panel_states[panel_num] = state


    def set_brightness(self, bright):
        self._global_brightness = brightness

    def __del__(self):
        self._is_running = False
        self._anim_update_thread.join()



class HardwareAPA102Controller(ControllerInterface):
    '''
    Hardware abstraction of APA102 LEDs used in Panther robot.
    '''

    LED_SWITCH_FRONT_STATE = True   # High - front panel | Low - tail panel
    LED_POWER_ON_STATE     = False  # active LEDs with low state 
    GLOBAL_MAX_BRIGHTNESS  = 15 

    # Assign numbers to panels
    class Panel(Enum):
        FRONT = 0
        TAIL  = 1

    class ControllerError(Exception):
        '''
        Exception raised for errors in the HardwareAPA102Controller.

        Attributes:
            message - explanation of the error.
        '''

        def __init__(self, message='Panther lights error'):
            self.message = message
            super().__init__(self.message)


    def __init__(self,
                 num_led,
                 panel_count: Optional[int] = 2,
                 brightness: Optional[int] = GLOBAL_MAX_BRIGHTNESS,
                 led_switch_pin: Optional[int] = 20,
                 led_power_pin: Optional[int] = 26):

        self._num_led = num_led
        self._panel_count = panel_count
        self._global_brightness = brightness

        self._pixels = apa102.APA102(num_led=num_led, order="rgb", mosi=10, sclk=11, global_brightness=brightness)
        self._led_switch_pin = led_switch_pin
        self._led_power_pin = led_power_pin

        self._front_active = True
        self._tail_active = True

        # Setup and activate panels
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self._led_switch_pin, GPIO.OUT)
        GPIO.setup(self._led_power_pin, GPIO.OUT)
        GPIO.output((self._led_switch_pin, self._led_power_pin), (HardwareAPA102Controller.LED_SWITCH_FRONT_STATE, HardwareAPA102Controller.LED_POWER_ON_STATE))



    def set_panel(self, panel_num, panel_frame, brightness=None):
        '''
        panel_num (HardwareAPA102Controller.Panel): panel identifier.
        panel_frame (Array): new frame to set. Dimensions: (3, led count in single panel)
        '''
        assert 0 <= panel_num < self._panel_count
        assert np.shape(panel_frame)[1] == self._num_led and np.shape(panel_frame)[0] == 3

        if brightness is None:
            brightness = self._global_brightness

        # Select panel
        if panel_num == HardwareAPA102Controller.Panel.FRONT:
            if self._front_active:
                GPIO.output(self._led_switch_pin, HardwareAPA102Controller.LED_SWITCH_FRONT_STATE)
        elif panel_num == HardwareAPA102Controller.Panel.TAIL:
            if self._tail_active:
                GPIO.output(self._led_switch_pin, not HardwareAPA102Controller.LED_SWITCH_FRONT_STATE)
        else:
            raise HardwareAPA102Controller.ControllerError('panther lights have only two panels')

        # Set all leds in this panel
        for i in range(self._pixels.num_led):
            self._pixels.set_pixel_rgb(i, panel_frame[i], brightness)
        self._pixels.show()


    def set_panel_state(self, panel_num, state):
        if panel_num == HardwareAPA102Controller.Panel.FRONT:
            self._front_active = state
        elif panel_num == HardwareAPA102Controller.Panel.TAIL:
            self._tail_active = state

    
    def set_brightness(self, bright):
        self._global_brightness = brightness

    
    def __del__(self):
        '''
        Cleans all leds and turns off both panels.
        '''
        self._pixels.cleanup()
        GPIO.output(self._led_power_pin, not HardwareAPA102Controller.LED_POWER_ON_STATE) 





if __name__ == '__main__':
    '''
    Simple GUI test
    '''

    vled = VirtualLEDController(num_led=6, panel_count=2)

    frames = [[[0,0,0,0,0,255],[0,255,0,0,0,0],[0,0,0,0,0,0]],
              [[0,0,0,0,255,0],[0,255,0,0,0,0],[0,0,0,0,0,0]],
              [[0,0,0,255,0,0],[0,255,0,0,0,0],[0,0,0,0,0,0]],
              [[0,0,255,0,0,0],[0,255,0,0,0,0],[0,0,0,0,0,0]],
              [[0,255,0,0,0,0],[0,255,0,0,0,0],[0,0,0,0,0,0]],
              [[255,0,0,0,0,0],[0,255,0,0,0,0],[0,0,0,0,0,0]]]

    for frame in frames:
        vled.set_panel(1, frame)
        time.sleep(0.5)
        vled.set_panel(0, frame)
        time.sleep(0.5)