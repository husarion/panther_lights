#!/usr/bin/env python3

import os
import yaml
import copy

import rospy
from std_srvs.srv import SetBool

from panther_lights_controller.event import Event
from panther_lights_controller.controller import Controller
from panther_lights_controller.led_config_importer import LEDConfigImporter
from husarion_msgs.msg import LEDAnimation, LEDAnimationArr
from husarion_msgs.srv import LEDBrightness, LEDPanel, LEDSetId, LEDSetImageAnimation

class LightsControllerNode:
    
    def __init__(self):
        '''lights_controller_node class'''
        
        self._anim_queue = []
        self._interrupt = False
        self._current_animation = None

        rospy.init_node('lights_controller_node')

        self._animation_queue_pub = rospy.Publisher('lights/controller/queue', LEDAnimationArr, queue_size=10)
        self._set_lights_service = rospy.Service('lights/controller/set/id', LEDSetId, self._set_lights_callback)
        self._set_image_animation_service = rospy.Service('lights/controller/set/image', LEDSetImageAnimation, self._set_image_animation_callback)
        self._set_brightness_service = rospy.Service('lights/controller/brightness', LEDBrightness, self._brightness_callback)
        self._set_panel_state_service = rospy.Service('lights/controller/panel_state', LEDPanel, self._panel_state_callback)
        self._set_panel_state_service = rospy.Service('lights/controller/clear_panel', LEDPanel, self._clear_panel_callback)
        self._set_panel_state_service = rospy.Service('lights/controller/clear_queue', SetBool, self._clear_queue_callback)
        self._set_panel_state_service = rospy.Service('lights/controller/kill_current_anim', SetBool, self._kill_current_anim_callback)
        self._lights_controller_timer = rospy.Timer(rospy.Duration(0.05), self._lights_controller)

        self._config_path = rospy.get_param('config_path', '../../config/led_conf.yaml')
        self._global_brightness = rospy.get_param('global_brightness', 1)
        self._num_led = rospy.get_param('num_led', 46)


        src_path = os.path.dirname(os.path.abspath(__file__))
        conf_path = os.path.join(src_path, self._config_path)
        conf_file = open(conf_path,'r')
        conf_yaml = yaml.load(conf_file, Loader=yaml.Loader)
        conf_file.close()

        self._led_config_importer = LEDConfigImporter(yaml_file=conf_yaml,
                                                      global_brightness=self._global_brightness,
                                                      num_led=self._num_led)

        self._controller = Controller(num_led=self._num_led,
                                      brightness=self._global_brightness)


        rospy.loginfo(f'{rospy.get_name()} started')


    def _lights_controller(self, *args):
        '''executes rosnode'''
        if self._current_animation:
            if self._current_animation.finished:
                del self._current_animation
                self._current_animation = None
                self._publish_queue_state()

            # animation interrupts and saves previous
            if self._anim_queue and self._interrupt:
                self._interrupt = False
                self._current_animation.stop()
                if self._current_animation.percent_done > 0.9:
                    del self._current_animation
                else:
                    if self._current_animation.percent_done < 0.15:
                        self._current_animation.reset()
                    self._anim_queue.insert(1, self._current_animation)

                self._current_animation = self._anim_queue.pop(0)
                if not self._current_animation.spawned:
                    self._current_animation.spawn(self._controller)
                self._publish_queue_state()
                self._current_animation.run()
        else:
            # await new animations
            while not self._anim_queue:
                return

            # get, spawn and start new animation
            self._current_animation = self._anim_queue.pop(0)
            if not self._current_animation.spawned:
                self._current_animation.spawn(self._controller)
            self._publish_queue_state()
            self._current_animation.run()

    
    def _publish_queue_state(self):
        '''returns animations execution order'''
        animations_list = LEDAnimationArr()
        animations_list.header.stamp = rospy.Time.now()

        if self._current_animation:
            key = self._led_config_importer.event_key(id=self._current_animation.id)
            animations_list.queue.append(LEDAnimation(key[0], key[1]))

        for anim in self._anim_queue:
            key = self._led_config_importer.event_key(id=anim.id)
            animations_list.queue.append(LEDAnimation(key[0], key[1]))


        self._animation_queue_pub.publish(animations_list)
        

    def _set_lights_callback(self, anim):
        '''rosservice callback for /lights/set/id'''
        animation = None
        try:
            if anim.animation.id != 0 and not anim.animation.name:
                key = self._led_config_importer.event_key(id=anim.animation.id)
            elif anim.animation.id == 0 and anim.animation.name:
                key = self._led_config_importer.event_key(name=anim.animation.name)
            else:
                key = self._led_config_importer.event_key(id=anim.animation.id, name=anim.animation.name)
            animation = self._led_config_importer.get_event(key)
            animation.param = anim.param
        except Exception as e:
            return 'failed'

        if animation.interrupting:
            self._anim_queue.insert(0, animation)
            self._interrupt = True
        else:
            self._anim_queue.append(animation)

        return 'success'


    def _set_image_animation_callback(self, image):
        '''rosservice callback for /lights/set/image_animation'''
        if not image.image_front or not image.image_rear or image.duration == 0:
            return 'failed'

        animation = {
            'repeat':     1,
            'duration':   0,
            'keep_state': False,
            'image':      0
        }
        event = {
            'front': copy.copy(animation),
            'rear':  copy.copy(animation)
        }

        try:
            if image.color:
                event['front']['color'] = int(image.color, 16)
                event['rear']['color'] = int(image.color, 16)

            if image.repeat != 0:
                event['front']['repeat'] = image.repeat
                event['rear']['repeat'] = image.repeat

            if image.keep_state:
                event['front']['keep_state'] = True
                event['rear']['keep_state'] = True

            event['front']['duration'] = image.duration
            event['rear']['duration'] = image.duration

            event['front']['image'] = image.image_front
            event['rear']['image'] = image.image_rear

            animation = Event({'animation':event, 'id':0}, self._num_led, self._global_brightness)
            self._anim_queue.insert(0, animation)
            self._interrupt = True
            return 'success'
        except Exception as e:
            rospy.logerr(f'{e}')
            return 'failed'


    def _brightness_callback(self, msg):
        '''rosservice callback for /lights/brightness'''
        self._controller.set_brightness(msg.data)
        return f'brightness: {msg.data}'


    def _panel_state_callback(self, msg):
        '''rosservice callback for /lights/panel_state'''
        self._controller.set_panel_state(msg.panel, msg.data)
        return 'success'


    def _clear_panel_callback(self, msg):
        '''rosservice callback for /lights/panel_state'''
        self._controller.clear(msg.panel, msg.data)
        return 'success'


    def _clear_queue_callback(self, msg):
        '''removes all elements from animation queue'''
        if msg.data:
            try:
                self._anim_queue.clear()
                return (True, 'queue cleared')
            except Exception as e:
                pass
            return (False, 'failed to clear queue')
        return (True, 'queue not cleared as requested')


    def _kill_current_anim_callback(self, msg):
        '''removes currently running animation'''
        if msg.data:
            try:
                self._current_animation.stop()
                del self._current_animation
                self._current_animation = None
                return (True, 'animation killed')
            except Exception as e:
                pass
            return (False, 'failed to kill animation')
        return (True, 'animation not killed as requested')


def main():
    try:
        lights = LightsControllerNode()
        rospy.spin()
    except Exception as e:
        rospy.logerr(f'panther_lights_controller error: {e}')
        exit(1)


if __name__ == '__main__':
    main()