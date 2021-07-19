import time
import queue
import numpy as np
from enum import Enum
from typing import Optional
from threading import Thread, Lock

import matplotlib.pyplot as plt
import warnings
warnings.filterwarnings("ignore")

from .controller import Controller


class VirtualLEDController(Controller):
    def __init__(self, 
                 num_led,
                 panel_count: Optional[int] = 2,
                 brightness: Optional[int] = 255):
        '''virtual LED controller based on matplotlib'''

        self._num_led = num_led
        self._panel_count = panel_count
        self._global_brightness = brightness
        self._frame = np.zeros((panel_count, num_led, 3))
        self._panel_states = np.ones(panel_count).astype(np.bool)
        self._mask = (np.ones(num_led) * 0x0000FF).astype(np.uint16)

        self._queue = queue.Queue()
        self._is_running = True
        self._thread_finished = False
        self._plt_lock = Lock()
        self._lock = Lock()

        for i in range(self._panel_count):
            self.set_panel(i, [0x000000]*self._num_led)

        self._anim_update_thread = Thread(target=self._update_anim, daemon=True)
        self._anim_update_thread.start()


    def set_panel(self, panel_num, panel_frame, brightness: Optional[int] = None):
        '''sets panel_frame on panel_num led panel'''
        with self._lock:
            if self._panel_states[panel_num]:
                panel_frame = np.array(panel_frame).T

                self._frame[panel_num,:,0] = (np.uint32(panel_frame) >> 16) & (self._mask)
                self._frame[panel_num,:,1] = (np.uint32(panel_frame) >>  8) & (self._mask)
                self._frame[panel_num,:,2] = (np.uint32(panel_frame)      ) & (self._mask)

                with self._plt_lock:
                    self._queue.put(self._frame)


    def _update_anim(self):
        plt.ion()
        with self._lock:
            self._im = plt.imshow(self._frame, vmin=0, vmax=255, animated=True, interpolation='nearest')
        plt.xticks([]), plt.yticks([])

        with self._plt_lock:
            running = self._is_running

        while running:
            try:
                with self._plt_lock:
                    panel_frame = self._queue.get(block=True, timeout=0.000001)
            except queue.Empty:
                pass
            
            self._im.set_array(panel_frame.astype(np.uint8))
            plt.pause(0.0001)

            with self._plt_lock:
                running = self._is_running

        plt.ioff()
        plt.close()
        with self._plt_lock:
            self._thread_finished = True


    def set_panel_state(self, panel_num, state):
        '''locks or unlocks given panel'''
        if not state:
            self.clear_panel(panel_num)
        with self._lock:
            self._panel_states[panel_num] = state


    def set_brightness(self, brightness):
        '''sets panel brightness'''
        with self._lock:
            self._global_brightness = brightness

    
    def clear_panel(self, panel_num):
        '''clears panel'''
        self.set_panel(panel_num, [0]*self._num_led)


    def __del__(self):
        '''stops thread and closes matplotlib window'''
        with self._plt_lock:
            self._is_running = False
        finished = False
        while not finished:
            with self._plt_lock:
                finished = self._thread_finished
            time.sleep(0.00001)

        self._anim_update_thread.join()