# Panther Lights animation scheduler

Panther Lights Scheduler is dockerized ROS node deciding which animation will be shown at the moment.


## Parameters
- `~low_battery_margin` *(double, default: 0.2)* - charge of battery at which *low_battery* starts being displayed.
- `~low_battery_anim_delay` *(double, default: 30)* - seconds between *low_battery* animation.

- `~critical_battery_anim_delay` *(double, default: 10)* - seconds between *critical_battery* animation.

## Subscribed topics

- `/battery` - battery status.
- `/panther_driver/manager/status` - Panther status information.
- `/move_base/status` - status of autonomous goals. 
- `/lights/controller/queue` - current animation queue.

## Published services
- `/lights/controller/set/id`
- `/lights/controller/panel_state`
- `/lights/controller/queue/clear`
- `/lights/controller/queue/kill_current`
- `/lights/controller/queue/remove_id`

## Animations
*panther_lights_scheduler* is expecting ROS service with name `/lights/controller/set/id` with message type *(husarion_msgs/LEDAnimationId)*.

Expects following animations defined:
- *id*: `1`, *name*: `'E-STOP'`
- *id*: `2`, *name*: `'ERROR'`
- *id*: `3`, *name*: `'GOAL_ARCHIVED'`
- *id*: `4`, *name*: `'CRITICAL_BATTERY'`
- *id*: `5`, *name*: `'LOW_BATTERY'`
- *id*: `6`, *name*: `'SHOW_BATTERY'`
- *id*: `7`, *name*: `'AUTONOMOUS_ACTION'`
- *id*: `8`, *name*: `'MANUAL_ACTION'`
- *id*: `9`, *name*: `'READY'`



