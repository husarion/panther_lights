import logging
from threading import Thread
import time
import queue


class PantherLightsQueueElem:
    def __init__(self, executor, is_background=False, interrupt=False):
        self._executor = executor
        self._is_background = is_background
        self._interrupt = interrupt

    @property
    def executor(self):
        return self._executor

    @property
    def is_background(self):
        return self._is_background

    @property
    def interrupt(self):
        return self._interrupt


class PantherLightsAnimationExecutorThread(Thread):

    def __init__(self,
                 queue,
                 driver,
                 time_step=0.01):

        '''Initialize PantherLights thread'''
        super().__init__(name="panther_lights_thread")

        self._queue = queue
        self._time_step = time_step
        self._driver = driver

        self._is_running = True
        self._background_animation = []
        self._switch_background_animation = False
        self._animation_queue = []


    def join(self):
        self._is_running = False
   
   
    def run(self):
        logging.info("Thread %s: started!", self._name)

        while self._is_running:
            start_time = time.time()
            self._queue_reader()

            if self._animation_queue:
                frame_front = self._animation_queue[0].executor(0)
                if frame_front is not None:
                    self._driver.set_panel(0, frame_front)

                frame_tail = self._animation_queue[0].executor(1)
                if frame_tail is not None:
                    self._driver.set_panel(1, frame_tail)
                elif frame_front is None and frame_tail is None:
                    self._animation_queue.remove(self._animation_queue[0])

            elif self._background_animation:
                frame_front = self._background_animation[0].executor(0)
                if frame_front is not None:
                    self._driver.set_panel(0, frame_front)

                frame_tail = self._background_animation[0].executor(1)
                if frame_tail is not None:
                    self._driver.set_panel(1, frame_tail)

                elif frame_front is None and frame_tail is None:
                    if self._switch_background_animation:
                        self._background_animation.remove(self._background_animation[0])
                        self._switch_background_animation = False
                    else:
                        self._background_animation[0].executor.reset()

            finish_time = time.time()
            time.sleep(self._time_step - (finish_time - start_time))


    def _queue_reader(self):
        try:
            anim = self._queue.get(block=True, timeout=self._time_step/30)
            if anim.interrupt:
                self._animation_queue.clear()
                
            if anim.is_background:
                if self._background_animation:
                    self._switch_background_animation = True
                self._background_animation.append(anim)

            else:
                self._animation_queue.append(anim)
                if self._switch_background_animation:
                    if self._background_animation:
                        self._background_animation.remove(self._background_animation[0])
                    self._switch_background_animation = False
                else:
                    if self._background_animation:
                        self._background_animation[0].executor.reset()
        except queue.Empty:
            pass
            