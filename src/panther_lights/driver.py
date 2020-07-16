from apa102_pi.driver import apa102 
import RPi.GPIO as GPIO
import logging
from threading import Thread, Event, Lock
from enum import Enum
import sys
import time
from queue import Queue

class Animation:
 
    class Type(Enum):
        NO_ANIMATION = 0
        FADE_COLOR = 1
        SOLID_COLOR = 2
        SNAKE_RIGHT_FAST = 3
        SNAKE_RIGHT_NORMAL = 4
        SNAKE_RIGHT_SLOW = 5
        SNAKE_LEFT_FAST = 6
        SNAKE_LEFT_NORMAL = 7
        SNAKE_LEFT_SLOW = 8
        SNAKE_HALF_RIGHT_FAST = 9
        SNAKE_HALF_LEFT_FAST = 10
        DOUBLE_SNAKE_NORMAL = 11
        DOUBLE_SNAKE_FAST = 12


    def __init__(self, pixels, default_animation=Type.NO_ANIMATION.value, default_color=0):
        self.pixels = pixels
        self.frame_count = 0
        self.bright_percent = 100 
        self.current_animation = default_animation
        self.animation_color = default_color

        self.even = True
        self.fade_steps_per_cycle = 30
        self.snake_index = 0
        self.snake_wait = 6
        self.double_snake_index = [0,0]
    
    def update(self):
        # compute new animation frame
        if self.current_animation == Animation.Type.FADE_COLOR.value:
            return self.fade_color()  
        elif self.current_animation == Animation.Type.SOLID_COLOR.value:
            return self.solid_color()
        elif self.current_animation == Animation.Type.SNAKE_RIGHT_SLOW.value:
            return self.snake(True,3,False)
        elif self.current_animation == Animation.Type.SNAKE_RIGHT_NORMAL.value:
            return self.snake(True,1,True, False)
        elif self.current_animation == Animation.Type.SNAKE_RIGHT_FAST.value:
            return self.snake(True,3,True, False)
        elif self.current_animation == Animation.Type.SNAKE_LEFT_SLOW.value:
            return self.snake(False,3,False, False)
        elif self.current_animation == Animation.Type.SNAKE_LEFT_NORMAL.value:
            return self.snake(False,1,True, False)
        elif self.current_animation == Animation.Type.SNAKE_LEFT_FAST.value:
            return self.snake(False,3,True, False)
        elif self.current_animation == Animation.Type.SNAKE_HALF_LEFT_FAST.value:
            return self.snake(False,2,True, True)
        elif self.current_animation == Animation.Type.SNAKE_HALF_RIGHT_FAST.value:
            return self.snake(True,2,True, True)
        elif self.current_animation == Animation.Type.DOUBLE_SNAKE_NORMAL.value:
            return self.double_snake(2)
        elif self.current_animation == Animation.Type.DOUBLE_SNAKE_FAST.value:
            return self.double_snake(3)

    def fade_color(self):
        frame = self.frame_count % self.fade_steps_per_cycle
        if frame <= self.fade_steps_per_cycle // 2:
            fade = 2.0 / self.fade_steps_per_cycle * frame
        else:
            fade = 2.0 - (2.0 / self.fade_steps_per_cycle * frame)

        red = int(((self.animation_color >> 16) & 0xff) * fade)
        green = int(((self.animation_color >> 8) & 0xff) * fade)
        blue = int((self.animation_color & 0xff) * fade)

        for i in range(self.pixels.num_led):
            if self.even and i % 2 == 0:
                self.pixels.set_pixel_rgb(i, 0x000000, bright_percent=self.bright_percent)
            else:
                self.pixels.set_pixel(i, red, green, blue, bright_percent=self.bright_percent)
        self.frame_count += 1
        self.pixels.show()

    def solid_color(self):
        if self.frame_count == 0:
            for i in range(self.pixels.num_led):
                if self.even:
                    self.pixels.set_pixel_rgb(i, self.animation_color if i % 2 == 0 else 0x000000, bright_percent=self.bright_percent)
                else:
                    self.pixels.set_pixel_rgb(i, self.animation_color, bright_percent=self.bright_percent)
            self.frame_count += 1
            self.pixels.show()

    def snake(self, direction, speed, mode, half=False):
        if self.frame_count == 0:
            if half:
                self.snake_index = self.pixels.num_led // 2 - 1 # delta
            else: 
                self.snake_index = 0 if direction else self.pixels.num_led-1 # delta 
            for i in range(self.pixels.num_led):
                self.pixels.set_pixel_rgb(i, 0x000000, bright_percent=self.bright_percent)

        if direction:
            if self.snake_index < self.pixels.num_led: 
                if mode:
                    end_index = self.pixels.num_led if self.snake_index + speed > self.pixels.num_led else self.snake_index + speed 
                    for i in range(self.snake_index, end_index):
                            self.pixels.set_pixel_rgb(i, self.animation_color, bright_percent=self.bright_percent)
                    self.snake_index = end_index
                elif self.frame_count % speed == 0:
                    self.pixels.set_pixel_rgb(self.snake_index, self.animation_color, bright_percent=self.bright_percent)
                    self.snake_index += 1
            elif self.snake_index - self.pixels.num_led < self.snake_wait:
                self.snake_index += 1
            elif self.snake_index - self.pixels.num_led == self.snake_wait:
                for i in range(self.pixels.num_led):
                    self.pixels.set_pixel_rgb(i, 0x000000, bright_percent=self.bright_percent)   
                self.snake_index += 1         
            elif self.snake_index - self.pixels.num_led < 2 * self.snake_wait:
                self.snake_index += 1
            else:
                self.snake_index = self.pixels.num_led // 2 - 1 if half else 0
        else:
            if self.snake_index >= 0:
                if mode:
                    end_index = -1 if self.snake_index - speed < 0 else self.snake_index - speed 
                    for i in range(self.snake_index, end_index, -1):
                            self.pixels.set_pixel_rgb(i, self.animation_color, bright_percent=self.bright_percent)
                    self.snake_index = end_index
                elif self.frame_count % speed == 0:
                    self.pixels.set_pixel_rgb(self.snake_index, self.animation_color, bright_percent=self.bright_percent)
                    self.snake_index -= 1
            elif self.snake_index >= - self.snake_wait-1:
                self.snake_index -= 1
            elif self.snake_index == -self.snake_wait-2:
                for i in range(self.pixels.num_led):
                    self.pixels.set_pixel_rgb(i, 0x000000, bright_percent=self.bright_percent)
                self.snake_index -= 1
            elif self.snake_index > (-self.snake_wait-1) * 2:
                self.snake_index -= 1
            else:
                self.snake_index = self.pixels.num_led // 2 - 1 if half else self.pixels.num_led - 1

        self.frame_count += 1
        self.pixels.show()

    def double_snake(self, speed):
        if self.frame_count == 0:
            for i in range(self.pixels.num_led):
                self.pixels.set_pixel_rgb(i, 0x000000, bright_percent=self.bright_percent)
            self.bright_percent = 70
            for i in range(2):
                self.double_snake_index[i] = self.pixels.num_led // 2 - 1
            self.pixels.set_pixel_rgb(self.double_snake_index[i], self.animation_color, bright_percent=self.bright_percent)
            self.pixels.show()
        elif self.double_snake_index[1] < self.pixels.num_led:
            for i in range(self.double_snake_index[0], self.double_snake_index[1]):
                self.pixels.set_pixel_rgb(i, self.animation_color, bright_percent=self.bright_percent)
            tmp = self.double_snake_index[0] - speed
            self.double_snake_index[0] = tmp if tmp >= 0 else 0
            tmp = self.double_snake_index[1] + speed
            self.double_snake_index[1] = tmp if tmp < self.pixels.num_led else self.pixels.num_led
            self.pixels.show()
        elif self.double_snake_index[1] < self.pixels.num_led + 3:
            self.bright_percent += 10
            self.double_snake_index[1] += 1
            for i in range(0,self.pixels.num_led):
                self.pixels.set_pixel_rgb(i, self.animation_color, bright_percent=self.bright_percent)
            self.pixels.show()
        self.frame_count += 1
        

class PantherLights(Thread):
    
    class Message:
        def __init__(self, anim_type_front, anim_color_front, anim_type_rear, anim_color_rear):
            self.anim_type = (anim_type_front, anim_type_rear)
            self.anim_color = (anim_color_front, anim_color_rear)
   
    DELTA_TIME = 0.033
    LED_SWITCH_FRONT_STATE = True 
    LED_POWER_ON_STATE = False # active with low state 
    GLOBAL_MAX_BRIGHTNESS = 15 
    
    def __init__(self, event, queue, num_leds=73, led_switch_pin=20, led_power_pin = 26, brightness = GLOBAL_MAX_BRIGHTNESS, rear_enabled = False):
        '''Initialize PantherLights thread'''
        super().__init__(name="panther_lights_thread")
        self.__led_switch_pin = led_switch_pin
        self.__led_power_pin = led_power_pin
        # initialize apa102 module and SPI outputs
        self.__pixels = apa102.APA102(num_led=num_leds, order="rgb", mosi=10, sclk=11, global_brightness=brightness)
        # initialize gpio outputs 
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.__led_switch_pin, GPIO.OUT)
        GPIO.setup(self.__led_power_pin, GPIO.OUT)
        GPIO.output((self.__led_switch_pin, self.__led_power_pin), (PantherLights.LED_SWITCH_FRONT_STATE, PantherLights.LED_POWER_ON_STATE))
        self.__stop_event = event
        self.__queue = queue
        self.__animation_front = Animation(self.__pixels)
        self.__animation_rear = Animation(self.__pixels)
        self.__front_enabled = True
        self.__rear_enabled = rear_enabled 
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
            new_msg = None

            # check the queue
            if not self.__queue.empty():
                new_msg = self.__queue.get(block=False)
                for i, animation in enumerate((self.__animation_front, self.__animation_rear)):
                    if new_msg.anim_type[i] in Animation.Type.__members__.keys():
                        animation.current_animation = Animation.Type[new_msg.anim_type[i]].value
                    animation.animation_color = new_msg.anim_color[i]
                    animation.frame_count = 0

            # if new_msg is not None:
            #     del new_msg

            # display new frame
            self.__lock.acquire()
            if self.__front_enabled:
                GPIO.output(self.__led_switch_pin, PantherLights.LED_SWITCH_FRONT_STATE)
                self.__animation_front.update()
            if self.__rear_enabled:
                GPIO.output(self.__led_switch_pin, not PantherLights.LED_SWITCH_FRONT_STATE)
                self.__animation_rear.update()
            self.__lock.release()
            time.sleep(PantherLights.DELTA_TIME) 
        logging.info("Thread %s: closing!", self._name) 
        self.__pixels.cleanup()
        GPIO.output(self.__led_power_pin, not PantherLights.LED_POWER_ON_STATE) 
        GPIO.cleanup()


def main():
    format = "%(asctime)s: %(message)s"
    stop_thread_event = Event()
    stop_thread_event.clear()
    my_queue = Queue()
    logging.basicConfig(format=format, level=logging.INFO, datefmt="%H:%M:%S")
    lights = PantherLights(event=stop_thread_event, queue=my_queue)
    lights.start()
    
    while 1:
        try:
            my_queue.put(PantherLights.Message('FADE_COLOR', 0xEEDD00, None, None))
            time.sleep(4)
            my_queue.put(PantherLights.Message('SNAKE_HALF_LEFT_FAST', 0xff6600, None, None))
            time.sleep(4)
            my_queue.put(PantherLights.Message('SNAKE_HALF_RIGHT_FAST', 0xff6600, None, None))
            time.sleep(4)
            my_queue.put(PantherLights.Message('DOUBLE_SNAKE_FAST', 0xff0000, None, None))
            time.sleep(4)
        except KeyboardInterrupt:
            stop_thread_event.set()
            lights.join()
            break
    logging.info("Closing program...")

if __name__=="__main__":
    main()
