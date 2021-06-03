import logging
from threading import Thread, Lock
import time
import queue


class AnimationLock:
    def __init__(self):
        self._lock = Lock()
        self._animation = None

    def set_executor(self, animation):
        with self._lock:
            self._animation = animation

    def get_executor(self):
        if not self._lock.locked():
            with self._lock:
                exec = self._animation 
                self._animation = None
            return exec
        return None


class PantherLightsAnimationExecutorThread(Thread):

    def __init__(self,
                 queue,
                 emergency_lock,
                 interrupt_lock,
                 driver,
                 time_step=0.01):

        '''Initialize PantherLights thread'''
        super().__init__(name='panther_lights_thread')

        self._queue = queue
        self._emergency_lock = emergency_lock
        self._interrupt_lock = interrupt_lock
        self._time_step = time_step
        self._driver = driver

        self._is_running = True
        self._background_animation = []
        self._switch_background_animation = False
        self._executor_queue = []


    def join(self):
        self._is_running = False
   
   
    def run(self):
        logging.info("Thread %s: started!", self._name)

        while self._is_running:
            start_time = time.time()
            self._queue_reader()

            if self._executor_queue:
                frame_front = self._executor_queue[0](0)
                if frame_front is not None:
                    self._driver.set_panel(0, frame_front)

                frame_tail = self._executor_queue[0](1)
                if frame_tail is not None:
                    self._driver.set_panel(1, frame_tail)

                if frame_front is None and frame_tail is None:
                    del self._executor_queue[0]

            finish_time = time.time()
            time.sleep(abs(self._time_step - (finish_time - start_time)))


    def _queue_reader(self):
        emergency_anim_executor = self._emergency_lock.get_executor()
        if emergency_anim_executor is not None:
            while not self._queue.empty():
                self._queue.get()
            self._executor_queue.clear()
            self._executor_queue.append(emergency_anim_executor)
            return
        
        interrupt_anim_executor = self._interrupt_lock.get_executor()
        if interrupt_anim_executor is not None:
            if self._executor_queue:
                if self._executor_queue[0].percent_done >= 0.75:
                    del self._executor_queue[0]
                else:
                    self._executor_queue[0].reset()
            self._executor_queue.insert(0, interrupt_anim_executor)
            return

        try:
            executor = self._queue.get(block=True, timeout=self._time_step/30)
            self._executor_queue.append(executor)
        except queue.Empty:
            pass
            