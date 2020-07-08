#!/usr/bin/env python3 
from apa102_pi.driver import apa102 
import RPi.GPIO as GPIO
import logging
from threading import Thread, Event, Lock
import sys
import time
from queue import Queue

class Animation:
 
    NO_ANIMATION = 0
    FADE_COLOR = 1
    SOLID_COLOR = 2
    NO_COLOR = 0 

    def __init__(self, buffer_size, default_animation=NO_ANIMATION, default_color=NO_COLOR):
        self.buffer = [0] * buffer_size
        self.frame_count = 0
        self.brightness = 31 
        self.current_animation = default_animation
        self.animation_color = default_color
    
    def update(self, animation_type=None, animation_color=None):
        if animation_type is None:
            animation_type = self.current_animation
        if animation_color is None:
            animation_color = self.animation_color

        # compute new animation frame
        if animation_type == Animation.FADE_COLOR:
            return self.fade_color(animation_color)  
        elif animation_type == Animation.SOLID_COLOR:
            return self.solid_color(animation_color)

         
    def fade_color(self, color):
        if self.current_animation == Animation.FADE_COLOR and self.animation_color == color:
            index = self.frame_count % 61 + 1 
            if index < 31: 
                self.brightness = index 
            else:
                self.brightness = 62 - index
            self.frame_count += 1
        else:
            self.current_animation = Animation.FADE_COLOR
            self.animation_color = color
            for i in range(len(self.buffer)):
                self.buffer[i] = self.animation_color
            self.frame_count = 0
            self.brightness = 0 
        return True
    
    def solid_color(self, color):
        if self.current_animation == Animation.SOLID_COLOR and self.animation_color == color:
            return False
        else:
            self.current_animation = Animation.SOLID_COLOR
            self.animation_color = color
            for i in range(len(self.buffer)):
                self.buffer[i] = self.animation_color
            return True
 
class PantherLights(Thread):
    
    class Message:
        def __init__(self, anim_type_front, anim_color_front, anim_type_rear, anim_color_rear):
            self.anim_type_front = anim_type_front
            self.anim_type_rear = anim_type_rear
            self.anim_color_front = anim_color_front
            self.anim_color_rear = anim_color_rear
   
    DELTA_TIME = 0.033
    
    def __init__(self, event, queue, num_leds=4, gpio_pin=22, gpio_state_front=True):
        super().__init__(name="panther_lights_thread")
        self.__gpio_state_front = gpio_state_front
        self.__gpio_pin = gpio_pin
        self.__pixels = apa102.APA102(num_led=num_leds, order="bgr", mosi=10, sclk=11)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.__gpio_pin, GPIO.OUT)
        GPIO.output(self.__gpio_pin, self.__gpio_state_front)
        self.__stop_event = event
        self.__queue = queue
        self.__animation_front = Animation(num_leds)
        self.__animation_rear = Animation(num_leds)
        self.__front_enabled = True
        self.__rear_enabled = True 
        self.__lock = Lock()

    @property
    def front_enabled(self):
        return self.__front_enabled

    @property
    def rear_enabled(self):
        return self.__rear_enabled
     
    def enable_strip(self, front = True, rear = True):
        self.__lock.acquire()
        self.__front_enabled = front
        self.__rear_enabled = rear
        self.__lock.release()
   
    def run(self):
        logging.info("Thread %s: started!", self._name)
        while not self.__stop_event.is_set():
            update_frame_front = False
            update_frame_rear = False
            new_msg = None

            # check the queue
            if not self.__queue.empty():
                new_msg = self.__queue.get(block=False)

            # compute new frame
            if new_msg is None: 
                update_frame_front = self.__animation_front.update() 
                update_frame_rear = self.__animation_rear.update()
            else:
                update_frame_front = self.__animation_front.update(new_msg.anim_type_front, new_msg.anim_color_front)
                update_frame_rear = self.__animation_rear.update(new_msg.anim_type_rear, new_msg.anim_color_rear) 
            
            if new_msg is not None:
                del new_msg

            # display new frame
            self.__lock.acquire()
            GPIO.output(self.__gpio_pin, self.__gpio_state_front)
            if update_frame_front and self.__front_enabled:
                for i in range(self.__pixels.num_led):
                    self.__pixels.set_pixel_rgb(i, self.__animation_front.buffer[i])
                self.__pixels.global_brightness = self.__animation_front.brightness 
                self.__pixels.show()
            GPIO.output(self.__gpio_pin, not self.__gpio_state_front)
            if update_frame_rear and self.__rear_enabled:
                for i in range(self.__pixels.num_led):
                    self.__pixels.set_pixel_rgb(i, self.__animation_rear.buffer[i])
                self.__pixels.global_brightness = self.__animation_rear.brightness
                self.__pixels.show()
            self.__lock.release()
            time.sleep(PantherLights.DELTA_TIME) 
        logging.info("Thread %s: closing!", self._name) 
        GPIO.cleanup()
        self.__pixels.cleanup()

def main():
    format = "%(asctime)s: %(message)s"
    stop_thread_event = Event()
    stop_thread_event.clear()
    my_queue = Queue()
    logging.basicConfig(format=format, level=logging.INFO, datefmt="%H:%M:%S")
    lights = PantherLights(event=stop_thread_event, queue=my_queue)
    lights.start()
    lights.enable_strip(True,False)
    
    while 1:
        try:
            my_queue.put(PantherLights.Message(Animation.FADE_COLOR, 0x0000ff, None, None))
            time.sleep(10)
            my_queue.put(PantherLights.Message(Animation.FADE_COLOR, 0xff0000, None, None))
            time.sleep(10)
            my_queue.put(PantherLights.Message(Animation.SOLID_COLOR, 0x00ff00, None,None))
            time.sleep(10)
        except KeyboardInterrupt:
            stop_thread_event.set()
            lights.join()
            break
    logging.info("Closing program...")

if __name__=="__main__":
    main()
