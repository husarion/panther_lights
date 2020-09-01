# panther_lights
Control Panther's APA102C lights using ROS.

version: 0.1.0

## Prerequisites

### `RPi.GPIO`
Installation:
```bash
$ pip3 install RPi.GPIO
```
Check if `gpio` group exist:
```
$ groups
```
If there's no `gpio` group, create it:
```bash
$ sudo groupadd gpio
```
and add current user to this group:
```bash
$ sudo adduser $USER gpio
```
Check if `/dev/gpiomem` exists and has correct permissions:
```bash
$ ls -l /dev/gpiomem
crw-rw---- 1 root gpio 244, 0 Dec 28 22:51 /dev/gpiomem
```
If it doesn’t then set the correct permissions as follows:
```bash
sudo chown root.gpio /dev/gpiomem
sudo chmod g+rw /dev/gpiomem
```
### `apa102_pi`
Installation:
```bash
$ git clone https://github.com/byq77/apa102-pi.git
$ cd apa102-pi && sudo python3 setup.py install
```

Check if spi is enabled
```
ls -l /dev | grep 'spi'
crw-------  1 root     root    153,   0 Apr  1 17:23 spidev0.0
crw-------  1 root     root    153,   1 Apr  1 17:23 spidev0.1
```
If there is no `spidev` files, add this line to `/boot/usercfg.txt`:
```plain
dtparam=spi=on
```
and restart the system.

Check if spi group exist:
```bash
$ groups
```
If there's no spi group, create it:
```bash
sudo groupadd spi
``` 
and add current user to it:
```bash
sudo adduser $USER spi
```
change `spidev0` ownership:
```bash
sudo chown root.spi /dev/spidev*
sudo chmod g+rw /dev/spidev*
```

### udev rules
In `/etc/udev/rules.d` create file `local.rules`:
```bash
$ sudo touch /etc/udev/rules.d/local.rules
```
Using `tee` add following lines:
```
$ sudo tee -a /etc/udev/rules.d/local.rules 
ACTION=="add", KERNEL=="spidev0.0", GROUP="spi", MODE="0660"
ACTION=="add", KERNEL=="gpiomem", GROUP="gpio", MODE="0660"
```
`Ctrl-D` to save

## Panther lights installation

Copy the package to your `ros_workspace/src` directory and in your workspace's root directory run:

```bash
$ catkin_make
$ catkin_make install
```

### ROS interface

To start node use:
```bash
$ rosrun panther_lights lights_node
```

To start led controller subscribing move_base status use:
```bash
$ rosrun panther_lights lights_controller_simple
```

To start launch composed of those two nodes use:
```bash
$ roslaunch panther_lights panther_lights_simple.launch
```


The node waits for service to `/set_panther_lights` topic with service type `/panther_lights/SetLights`:
```
uint8 BLINKER_RIGHT = 0
uint8 BLINKER_LEFT = 1
uint8 BRAKE_FRONT = 2
uint8 BRAKE_REAR = 3
uint8 BRAKE_BOTH = 4
uint8 NORMAL_FORWARD = 5
uint8 NORMAL_REVERSING = 6
uint8 SKID_RIGHT = 7
uint8 SKID_LEFT = 8
uint8 ERROR = 9
uint8 animation
string custom_color
```

Change animation to `BLINIKER_LEFT`:

```bash
$ rosservice call /set_panther_lights "animation: 1
custom_color: ''"
```

Change animation to `BLINKER_LEFT` with custom colors (front is green, rear is red):

```bash
$ rosservice call /set_panther_lights "animation: 1
custom_color: '0x00FF00 0xFF0000'"
```

### Animation configuration

You can customize your animation in file `panther_lights_animations.yaml`. You can find it in your installation directory (`<installation_path>install/share/config/panther_lights_animations.yaml`). 

Default configuration:
```yaml
# animations.yaml
# BLINKER_RIGHT = 0
# BLINKER_LEFT = 1
# BRAKE_FRONT = 2
# BRAKE_REAR = 3
# BRAKE_BOTH = 4
# NORMAL_FORWARD = 5
# NORMAL_REVERSING = 6
# SKID_RIGHT = 7
# SKID_LEFT = 8
# ERROR = 9
global_brightness: 20
num_leds: 73

animations:
  - id: 0 # BLINKER_RIGHT
    front: {type: 'COLOR_WIPE_HALF_DOWN_FAST', color: !!int 0xff6600}
    back: {type: 'COLOR_WIPE_HALF_UP_FAST', color: !!int 0xff6600}
  
  - id: 1 # BLINKER_LEFT
    front: {type: 'COLOR_WIPE_HALF_UP_FAST', color: !!int 0xff6600}
    back: {type: 'COLOR_WIPE_HALF_DOWN_FAST', color: !!int 0xff6600}
  
  - id: 2 # BRAKE_FRONT
    front: {type: 'DOUBLE_COLOR_WIPE_FAST', color: !!int 0xff0000}
    back: {type: 'SOLID_COLOR', color: !!int 0xffffff}
  
  - id: 3 # BRAKE_REAR
    front: {type: 'SOLID_COLOR', color: !!int 0xffffff}
    back: {type: 'DOUBLE_COLOR_WIPE_FAST', color: !!int 0xff0000}
  
  - id: 4 # BRAKE_BOTH
    front: {type: 'DOUBLE_COLOR_WIPE_FAST', color: !!int 0xff0000}
    back: {type: 'DOUBLE_COLOR_WIPE_FAST', color: !!int 0xff0000}

  - id: 5 # NORMAL_FORWARD
    front: {type: 'SOLID_COLOR', color: !!int 0xffffff}
    back: {type: 'SOLID_COLOR', color: !!int 0xffffff}

  - id: 6 # NORMAL_REVERSING
    front: {type: 'SOLID_COLOR', color: !!int 0xffffff}
    back: {type: 'FADE_COLOR', color: !!int 0xEEDD00}

  - id: 7 # SKID_RIGHT
    front: {type: 'COLOR_WIPE_DOWN_NORMAL', color: !!int 0xEEDD00}
    back: {type: 'COLOR_WIPE_UP_NORMAL', color: !!int 0xEEDD00}

  - id: 8 # SKID_LEFT
    front: {type: 'COLOR_WIPE_UP_NORMAL', color: !!int 0xEEDD00}
    back: {type: 'COLOR_WIPE_DOWN_NORMAL', color: !!int 0xEEDD00}

  - id: 9 # ERROR
    front: {type: 'FADE_COLOR', color: !!int 0xFF0000}
    back: {type: 'FADE_COLOR', color: !!int 0xFF0000}
```
