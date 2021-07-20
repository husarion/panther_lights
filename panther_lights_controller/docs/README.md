# Defining own events and animations


## *config.yaml*
Config file specifies paths to yaml files containing animations defined by Husarion and custom user animations.
```yaml
event_animations_files:
  - panther_events.yaml
  - my_custom_events.yaml
  - /home/user/my_another_custom_events.yaml
```
If paths specified in those files aren't global paths this node will search for them in */config* directory of package. If path is global, the config file will be read from outside of the package scope.

Animations will be read from specified files. If two files contain animations with the same id or name, the program will throw an exception and stop.

## Event file
```yaml
events:
  - id: 1
    name: 'EXAMPLE_1'
    interrupting: true
    animation:
      front:
        image: example_front.png
        duration: 3
        repeat: 3
      rear:
        image: example_rear.png
        duration: 2
        repeat: 1
  - id: 2
    name: 'EXAMPLE_2'
    animation:
      both:
        animation_name: example_code_animation
        duration: 4
        repeat: 2
        brightness: 0.5
```

This events.yaml file will create two available animations. You can execute them by running:
```
rosservice call /lights/controller/set/id "animation:
  id: 1
  name: 'EXAMPLE_1'
param: ''"
```
```
rosservice call /lights/controller/set/id "animation:
  id: 2
  name: 'EXAMPLE_2'
param: ''"
```

### Keywords
If the keyword doesn't have a default value this means it's obligatory to manually set it's value.

#### events block
- `id` *(int)* - positive, non zero unique animation identifier.
- `name` *(string)* - non-empty human readable animation name.
- `interrupting` *(bool, default false)* - flag specifying if animation can interrupt currently executed animation stopping it and pushing back to the queue.

#### animation block
- `animation` - groups animation definition.
- `front` - groups animation definition for front LED panel.
- `rear` - groups animation definition for rear LED panel.
- `both` - can be used in exchange for `rear` and `front`. Sets the same settings for both front and rear panels.

#### animation definition
- `image` *(string)* - name of *.png* file that will be displayed. By default image is being searched in */animation* directory of package. If the path is global, the image will be read from outside of the package scope.
- `animation_name` *(string)* - name of animation described in code.
- `duration` *(double)* - time of execution of animation. Value has to be greater than 0.
- `repeat` *(int, default 1)* - how many times the animation will be repeated. Value has to be greater than 0. Overall time of animation will be `repeat` times `duration`.
- `brightness` *(float, default global_brightness)* - overwrite the `global_brightness` variable.
- `color` *(int, default None)* - hexadecimal value of color. Has no default value and will be omitted if not specified. Overrides default color of animation.

You can only specify one source of animation. It has to be either `image` or `animation_name`. If animation needs to adapt in runtime depending on parameter you have to implement it in code and then refer to it using `animation_name`.

## Own animations

### Image animation
For most cases this will be the easiest solution to implement animation. Simply draw animation in Gimp, MS Paint or any other tool and save it. Then copy to the robot and link it's path.

Images have to have exactly the same width as the LED count set from `num_led`.

Color of animation can be changed by specifying `color` value. Then the image will be turned to grayscale, normalized and given color will be darkened depending on the grayscale value.

Animations will be displayed from top to bottom with time step between frames being equal to *image_height* divided by `duration`.

For animation defined with this image:
<div style="text-align:center">
<img src="./battery_orange-green.png" alt="drawing" height="400" width="400"/>
</div>

The output will look like this:
<div style="text-align:center">
<img src="./battery_orange-green.gif" alt="drawing" height="20" width="400"/>
</div>

### Code animations
In order to add own code animation write python script and add it to */src/panther_lights_controller/animations* folder. Inherit from class *Animation* and register it in *\_\_init\_\_.py*. Here is minimal animation class implementation. Animation will be recognized in *events.yaml* by `ANIMATION_NAME`. Param will be passed from *rosservice call* every time it is executed. It is passed to an already existing object on runtime.

```python
# MyAnimation.py
from .animation import Animation
import numpy as np

class MyAnimation(Animation):
    
    ANIMATION_NAME = 'my_animation'
    def __init__(self, anim_yaml, num_led, global_brightness, panel):
        '''my animation class'''
        super().__init__(anim_yaml, num_led, global_brightness, panel)
        self._num_led = num_led
        self._sleep_time = 0.01
    

    def __call__(self):
        '''returns new frame'''
        if self._i < self._cycle_steps:
            return [0]*self._num_led

        raise Animation.AnimationFinished

    @property
    def sleep_time(self):
        '''returns time needed to sleep in thread between frames'''
        return self._sleep_time

    def reset(self):
        '''restets animation to it's initial state'''
        pass

    def param(self, val):
        '''sets animations param'''
        pass
    param = property(None, param)
```

```python
# __init__.py
from .animation import Animation

from .your_animation import YourAnimation

BASIC_ANIMATIONS = {
        MyAnimation.ANIMATION_NAME : MyAnimation
}
```

