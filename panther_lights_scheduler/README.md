# Panther Lights animation scheduler

Panther Lights Scheduler is dockerized ROS node deciding which animation will be shown at the time. Node reads `/panther_driver/manager/status`. Based on that displays animation for given state. Here is list of animation and their meaning:

```
Place here animations
```

Autonomous action is integrated with *move_base* package and reads it's goal queue.

## Parameters
- `low_battery_margin` *(double, default 0.2)* - charge of battery at which *low_battery* starts being displayed.
- `low_battery_anim_delay` *(double, default 30)* - seconds between *low_battery* animation.

- `critical_battery_anim_delay` *(double, default 10)* - seconds between *critical_battery* animation.
