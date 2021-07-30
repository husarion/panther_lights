# Panther Lights hardware controller

Panther Lights Scheduler is dockerized ROS node deciding which animation will be shown at the time. Node reads `/panther_driver/manager/status`. Based on that displays animation for a given state. 

## Calling animation
In order to execute animation you have to call service `/lights/controller/set/id`. The call will look like this:
```
rosservice call /lights/controller/set/id "animation:
  id: 1
  name: 'E-STOP'
param: ''"
```


fields *id* and *name* are used to determinate animation described in *config.yaml*. In order to call animation you can only specify *id* or *name*. If both are specified node will check if they match.

*param* is being passed to animation during execution. Can be used for example to animate battery state on runtime. For example to show battery state equal 50% the call will lok like this:
```
rosservice call /lights/controller/set/id "animation:
  id: 6
  name: 'SHOW_BATTERY'
param: '0.5'"
```


## Services
- `/lights/controller/set/id` *(husarion_msgs/LEDAnimationId)* - executes animation read from config file.
- `/lights/controller/set/image` *(husarion_msgs/LEDSetImageAnimation)* - executes animation from specified panel on both panels.
- `/lights/controller/brightness` *(husarion_msgs/LEDBrightness)* - sets global brightness.
- `/lights/controller/panel_state` *(husarion_msgs/LEDPanel)* - turns on or off specified panel.
- `/lights/controller/clear_panel` *(husarion_msgs/LEDPanel)* - clears specified panel.
- `/lights/controller/queue/clear` *(std_srvs/SetBool)* - clears animation queue.
- `/lights/controller/queue/kill_current` *(std_srvs/SetBool)* - kills currently executed animation.
- `/lights/controller/queue/remove_id` *(husarion_msgs/LEDAnimationId)* - removes all animations with given ID from queue.


## Publish
- `lights/controller/queue`, *(husarion_msgs/LEDAnimationArr)* - current animation queue called every time the animation queue changes.


## Parameters
- `~config_path` *(string, default: '$(panther_lights_controller)/config/led_config.yaml)* - YAML file containing panel configuration.
- `~global_brightness` *(float, default: 1)* - global brightness ranges between 0 to 1.
- `~num_led` - *(int, default: 46)* - LED count for both panels.


## More instructions
More detailed instructions can be found in *README<span>.</span>md* files in following directories:
- how to configure animations - */docs*
- controller specific dependencies - */platforms/\<platform\>*

