# CARLA Dataset generator
This is a script that generates RGB, Segmentation and depth ground-truth data from Carla. This repository works with Carla version 0.9.6.  
### Instructions

```
0. Clone this repository under carla/PythonAPI folder.

```

```
1. Run the CARLA simulator

./CarlaUE4.sh -p 2000 -quality-level Epic

```

```
2. In a separate terminal run the python client

python client_render.py --port 2000 --town Town05 --weather_id 2 --camera_group ForwardCameras

```
Configuration.
Commandline arguments

```
camera_group: chooses which camera group to render. The choice should be within the keys in the 'cameras' dictionary defined in camera_configs.py file.
By default we have 3 camera groups. 'HorizontalCameras',
 'ForwardCameras', 'SideCameras'. There is the 'All',
 alternative that allows you to render with all camera groups at the same time.
```

```
town: choice of town to be rendered, you can choose 'Town01', 'Town02', 'Town03', 'Town04' or 'Town05'. default Town01
```

```
port: default=2000
```
```
weather_id: an integer to choose weather conditions, at the moment I am using 4 weather conditions numbered [0,1,2,3]
```

```
test_mode: this generates with a different random seed. Useful when you want to generate a sequence that is 'different' from other renderings. An example use case for turning on this flag is, when you want to generate a training and a test sequence by running two different simulations on the same city. In this case, you can run one simulation for training(without test_mode flag) and another for test(with test_mode flag).
```

Example usage for town 05 weather is 2 in test_mode:

```
python client_render.py --port 2000 --town Town05 --weather_id 2 --camera_group All --test_mode

```

TODO:
save camera pose at each frame.
