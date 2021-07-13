from threading import Thread, Lock, Event
import time

from .animation import Animation

class AnimationRuntime(Thread):

    def __init__(self, controller, animation):
        '''animation threading wrapper'''
        super().__init__()
        self._controller = controller
        self._animation = animation
        self._kill = False
        self._finished = False

        self._time_elapsed = 0
        self._panel = self._animation.panel

        self._cnt = 0

        self._lock = Lock()
        self._finished_lock = Lock()
        self._event = Event()
        self._event.clear()


    def run(self):
        '''animation thread main loop'''
        sleep_time = 0

        while self._cnt < self._animation.loops:
            while True:
                # for better time consistency sleep is corrected with exectution time
                start = time.time()
                self._event.wait()
                if self._kill:
                    return

                with self._lock:
                    try:
                        frame = self._animation()
                        sleep_time = self._animation.sleep_time
                    except Animation.AnimationFinished:
                        break

                self._controller.set_panel(self._panel, frame)
                block_time = abs(time.time() - start)
                loop_correction = 0
                time.sleep(min(sleep_time, abs(sleep_time-block_time)))
            self._cnt += 1

            with self._lock:
                self._animation.reset()

        with self._finished_lock:
            self._finished = True


    def kill(self):
        '''requests finishing the thread and wait for thread to stop'''
        with self._lock:
            self._kill = True
            if not self._event.is_set():
                self._event.set()


    def reset(self):
        '''restets animation to it's state at beginning of each loop'''
        with self._lock:
            self._cnt = 0
            self._animation.reset()


    def stop_animation(self):
        '''stops main animation loop'''
        with self._lock:
            if self._event.is_set():
                self._event.clear()
                self._time_elapsed += time.time() - self._start_time


    def run_animation(self):
        '''resumes main animation loop'''
        with self._lock:
            if not self._event.is_set():
                self._event.set()
                self._start_time = time.time()


    @property
    def brightness(self):
        '''get running animation brightness'''
        with self._lock:
            return self._animation.brightness


    @property
    def percent_done(self):
        '''get already passed percent of animation'''
        with self._lock:
            if self._event.is_set():
                percent = (self._time_elapsed + time.time() - self._start_time) \
                    / (self._animation.loops * self._animation.duration)
                return percent
            else:
                return (self._time_elapsed) / (self._animation.loops * self._animation.duration)


    @property
    def finished(self):
        '''get flag if main loop is finished'''
        with self._finished_lock:
            return self._finished
