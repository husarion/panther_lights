import copy

from .animations import AnimationRuntime, BASIC_ANIMATIONS

class Event:

    class EventError (Exception):
        '''basic error for event'''
        def __init__(self, message='YAML keyword error'):
            self.message = message
            super().__init__(self.message)

    def __init__(self, event_yaml, num_led, global_brightness):
        '''event combining front and rear animation'''
        self._id = event_yaml['id']

        if 'interrupting' in event_yaml.keys():
            self._interrupting = event_yaml['interrupting']
        else:
            self._interrupting = False

        self._front_animation = None
        self._rear_animation = None
        self._front_animation_runtime = None
        self._rear_animation_runtime = None

        if 'both' in event_yaml['animation'].keys():
            self._front_animation = self._match_animation_type(
                event_yaml['animation']['both'], num_led, global_brightness, 0)
            self._rear_animation = copy.deepcopy(self._front_animation)
            self._rear_animation.panel = 1

        elif 'front' in event_yaml['animation'].keys() and 'rear' in event_yaml['animation'].keys():
            self._front_animation = self._match_animation_type(
                event_yaml['animation']['front'], num_led, global_brightness, 0)
            self._rear_animation = self._match_animation_type(
                event_yaml['animation']['rear'], num_led, global_brightness, 1)
        else:
            missing_animations = (set(["front", "rear", "both"])) - set(event_yaml["animation"].keys())
            raise Event.EventError (f'no {missing_animations} in {event_yaml}')

    
    def _match_animation_type(self, animation_yaml, *args):
        '''matches if animation is based on image, or uses custom function'''
        if 'animation_name' in animation_yaml.keys():
            try:
                return BASIC_ANIMATIONS[animation_yaml['animation_name']](animation_yaml, *args)
            except KeyError:
                raise Event.EventError (f'no animation with name: {animation_yaml["animation_name"]}')
        elif 'image' in animation_yaml.keys():
            return BASIC_ANIMATIONS['image_animation'](animation_yaml, *args)
        else:
            raise Event.EventError (f'no matching animation type in {animation_yaml}')


    def spawn(self, controller):
        '''spawns both animations threads, animations has to be run separately'''
        self._front_animation_runtime = AnimationRuntime(controller, self._front_animation)
        self._rear_animation_runtime = AnimationRuntime(controller, self._rear_animation)
        
        self._front_animation_runtime.start()
        self._rear_animation_runtime.start()   


    def reset(self):
        '''resets both animations to initial state'''
        self._front_animation_runtime.reset()
        self._rear_animation_runtime.reset()


    def run(self):
        '''resumes both animations'''
        self._front_animation_runtime.run_animation()
        self._rear_animation_runtime.run_animation()


    def stop(self):
        '''stops both animations'''
        self._front_animation_runtime.stop_animation()
        self._rear_animation_runtime.stop_animation()
    

    def __del__(self):
        '''stops and destroys both animations threads'''
        if self._front_animation_runtime:
            self._front_animation_runtime.kill()
            self._front_animation_runtime.join()

        if self._rear_animation_runtime:
            self._rear_animation_runtime.kill()
            self._rear_animation_runtime.join()

    def param(self, val):
        '''sets animations param'''
        self._front_animation.param = val
        self._rear_animation.param = val
    param = property(None, param)
        

    @property
    def brightness(self):
        '''returns brightness of both animations as a tuple'''
        return (self._front_animation.brightness(), self._rear_animation.brightness())


    @property
    def interrupting(self):
        '''returns if event is interrupting'''
        return self._interrupting


    @property
    def percent_done(self):
        '''returns percentage done of longer animation'''
        return min(self._front_animation_runtime.percent_done, self._rear_animation_runtime.percent_done)


    @property
    def id(self):
        '''returns event ID'''
        return self._id

    @property
    def finished(self):
        '''returns if both animations finished'''
        return self._front_animation_runtime.finished and self._rear_animation_runtime.finished

    @property
    def spawned(self):
        '''returns if both animations were already spawned'''
        return self._front_animation_runtime is not None and self._rear_animation_runtime is not None