# panther_lights
Dockerized nodes used to control Panther LED panels and display display it's status. Package nodes:

- `panther_lights_controller` - ROS node controlling LED panels connected robot.

- `panther_lights_scheduler` - ROS node deciding which animation to show on LED panel.

`panther_lights_controller` can support various LED panels. Currently supports *APA102* based and virtual LED panel using matplotlib.


## Building a Docker image
#### panther_lights_controller
```bash
cd panther_lights_controller/panther_lights_controller_base
sudo docker build -t panther_lights_controller:base .
```
For given controller implementation run:
```bash
cd ../platforms/panther_lights_controller_<platform>
sudo docker build -t panther_lights_controller:<platform> .
```

#### panther_lights_scheduler
```bash
cd panther_lights_scheduler/panther_lights_scheduler
sudo docker build -t panther_lights_scheduler .
```

## Examples
Before running *docker-compose* build `panther_lights_controller:base` docker image from previous section
### GUI based demo using Matplotlib
```bash
cd examples/matplotlib_gui
xhost local:root
docker-compose up --build
```
### Raspberry Pi
```bash
cd examples/raspberry_pi
docker-compose up --build
```