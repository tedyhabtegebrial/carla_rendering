#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys
import tqdm
from copy import copy
import argparse

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import logging

import random

import time
import cv2 as cv
import numpy as np
from camera_configs import configs
from camera_configs import cameras as c_configs
try:
    import queue
except ImportError:
    import Queue as queue

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument('--town', type=str, default='Town01', help='choose [Town0x] x in [1,5]')
arg_parser.add_argument('--port', type=int, default=2000)
arg_parser.add_argument('--weather_id', type=int, default=0, help='choose [0,1,2,3]')
arg_parser.add_argument('--camera_group', type=str, default='All',
                        help='choice=[HorizontalCameras, ForwardCameras, SideCameras, All]')
arg_parser.add_argument('--test_mode', action='store_true', help='use when generating test dataset')
args = arg_parser.parse_args()

configs['town'] = args.town
configs['weather_id'] = args.weather_id
configs['port'] = args.port
if not args.camera_group in ['HorizontalCameras', 'ForwardCameras', 'SideCameras', 'All']:
    print('camera group should be in ', c_configs.keys())
if args.camera_group=='All':
    camera_configs = c_configs
else:
    camera_configs = {args.camera_group: c_configs[args.camera_group]}

TEST_SEQUENCES = args.test_mode
if TEST_SEQUENCES:
    print('Running in test mode')
    random.seed(123+100*(configs['weather_id']))
    configs['dest_path'] = configs['dest_path']+'_test'
else:
    print('Running in train mode')
    random.seed(1234+100*(configs['weather_id']))


SpawnActor = carla.command.SpawnActor
SetAutopilot = carla.command.SetAutopilot
FutureActor = carla.command.FutureActor


class CarlaSyncMode(object):
    """
    Context manager to synchronize output from different sensors. Synchronous
    mode is enabled as long as we are inside this context

        with CarlaSyncMode(world, sensors) as sync_mode:
            while True:
                data = sync_mode.tick(timeout=1.0)

    """

    def __init__(self, world, sensors, **kwargs):
        self.world = world
        self.sensors = sensors
        self.frame = None
        self.delta_seconds = 1.0 / kwargs.get('fps', 20)
        self._queues = []
        self._settings = None

    def __enter__(self):
        self._settings = self.world.get_settings()
        self.frame = self.world.apply_settings(carla.WorldSettings(
            no_rendering_mode=False,
            synchronous_mode=True,
            fixed_delta_seconds=self.delta_seconds))
        self.world.set_weather(_get_weather(configs['weather_id']))
        def make_queue(register_event):
            q = queue.Queue()
            register_event(q.put)
            self._queues.append(q)
        for sensor in self.sensors:
            make_queue(sensor.listen)
        return self

    def tick(self, timeout, get_data=False):
        self.frame = self.world.tick()
        if get_data:
            data = [self._retrieve_data(q, timeout) for q in self._queues]
            assert all(x.frame == self.frame for x in data)
        else:
            data = None
        return data

    def __exit__(self, *args, **kwargs):
        self.world.apply_settings(self._settings)

    def _retrieve_data(self, sensor_queue, timeout):
        while True:
            data = sensor_queue.get(timeout=timeout)
            if data.frame == self.frame:
                return data

get_converter = {}
get_converter['rgb'] = carla.ColorConverter.Raw
get_converter['depth'] = carla.ColorConverter.Raw
get_converter['semantic_segmentation'] = carla.ColorConverter.Raw

# if you want to see the renderings use the following converters
# get_converter['rgb'] = carla.ColorConverter.Raw
# get_converter['depth'] = carla.ColorConverter.LogarithmicDepth
# get_converter['semantic_segmentation'] = carla.ColorConverter.CityScapesPalette

def give_actors_start_and_end(world, all_id):
    all_actors = world.get_actors(all_id)
    for i in range(0, len(all_id), 2):
        # start walker
        all_actors[i].start()
        # set walk to random point
        all_actors[i].go_to_location(world.get_random_location_from_navigation())
        # random max speed
        all_actors[i].set_max_speed(1 + random.random()) # max speed between 1 and 2 (default is 1.4 m/s)

def get_single_camera(world, camera_bp, cam_configs, vehicle, i):
    x,y,z = cam_configs['x_locs'][i], cam_configs['y_locs'][i], cam_configs['z_locs'][i]
    camera_transform = carla.Transform(carla.Location(x=x, y=y, z=z),
                                       carla.Rotation(pitch=0.0, yaw=cam_configs['rot_yaw'], roll=0.0))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
    return camera

def get_camera_group(configs, cam_group, cam_configs, vehicle, blueprint_library, world, type='rgb'):
    rgb_cams = []
    dest_paths = []
    # converters = []
    weather_id = 'weather_' + str(configs['weather_id']).zfill(2)
    for ii in range(len(cam_configs['x_locs'])):
        camera_bp = world.get_blueprint_library().find('sensor.camera.'+type)
        world.wait_for_tick(3)
        # camera_bp.set_attribute('sensor_tick', '1.0')
        camera_bp.set_attribute('image_size_y', '600')
        camera_bp.set_attribute('image_size_x', '800')
        camera_bp.set_attribute('fov', '90')
        group_id = str(cam_group + '_' + str(ii).zfill(2))
        world.wait_for_tick(3)
        dest_folder = str(os.path.join(configs['dest_path'], configs['town'], weather_id, group_id, type))
        camera = get_single_camera(world, camera_bp, cam_configs, vehicle, ii)
        world.wait_for_tick(3)
        dest_paths.append(dest_folder)
        rgb_cams.append(camera)
    return rgb_cams, dest_paths

def get_cameras(configs, camera_configs, vehicle, bp, world):
    list_of_cameras, destination_paths, converters = [], [], []
    for cam_group, group_config in camera_configs.items():
        for cam_type in group_config['sensor_types']:
            cams, dest_paths = get_camera_group(configs, cam_group, group_config, vehicle, bp, world, cam_type)
            list_of_cameras.extend(cams)
            destination_paths.extend(dest_paths)
    return list_of_cameras, destination_paths

def _get_weather(id):
    id_to_weather = [carla.WeatherParameters.ClearNoon,
                    carla.WeatherParameters.Default,
                    carla.WeatherParameters.CloudyNoon,
                    carla.WeatherParameters.ClearSunset,]
    return id_to_weather[id]

def spawn_cars_new(configs, world, client):
    '''
    this is adapted from the CARLA PythonAPI tutorial scripts

    '''
    batch = []
    spawn_points = world.get_map().get_spawn_points()
    number_of_spawn_points = len(spawn_points)
    blueprint_library = world.get_blueprint_library()
    car_blueprints = blueprint_library.filter('vehicle')
    vehicles_list = []
    for n, transform in enumerate(spawn_points):
        if n >= configs['number_of_vehicles']:
            break
        blueprint = random.choice(car_blueprints)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        blueprint.set_attribute('role_name', 'autopilot')
        batch.append(SpawnActor(blueprint, transform).then(SetAutopilot(FutureActor, True)))
    for response in client.apply_batch_sync(batch):
        if response.error:
            logging.error(response.error)
        else:
            vehicles_list.append(response.actor_id)
    return vehicles_list


def spawn_walkers_new(configs, world, client):
    '''
    this is adapted from the CARLA PythonAPI tutorial scripts

    '''
    spawn_points = []
    walkers_list = []
    all_id = []
    blueprintsWalkers = world.get_blueprint_library().filter('walker')
    for i in range(configs['num_pedestrians']):
        spawn_point = carla.Transform()
        loc = world.get_random_location_from_navigation()
        world.wait_for_tick()
        if (loc != None):
            spawn_point.location = loc
            spawn_points.append(spawn_point)
    batch = []
    for spawn_point in spawn_points:
        walker_bp = random.choice(blueprintsWalkers)
        if walker_bp.has_attribute('is_invincible'):
            walker_bp.set_attribute('is_invincible', 'false')
        batch.append(SpawnActor(walker_bp, spawn_point))
    results = client.apply_batch_sync(batch, True)
    for i in range(len(results)):
        if results[i].error:
            logging.error(results[i].error)
        else:
            walkers_list.append({"id": results[i].actor_id})
    batch = []
    walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
    for i in range(len(walkers_list)):
        batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
        world.wait_for_tick()
    results = client.apply_batch_sync(batch, True)
    world.wait_for_tick()
    for i in range(len(results)):
        if results[i].error:
            logging.error(results[i].error)
        else:
            walkers_list[i]["con"] = results[i].actor_id
    # 4. we put altogether the walkers and controllers id to get the objects from their id
    for i in range(len(walkers_list)):
        all_id.append(walkers_list[i]["con"])
        all_id.append(walkers_list[i]["id"])
    # all_actors = world.get_actors(all_id)
    return all_id

def main():
    actor_list = []
    vehicles_list = []
    all_walkers = []
    # In this tutorial script, we are going to add a vehicle to the simulation
    # and let it drive in autopilot. We will also create a camera attached to
    # that vehicle, and save all the images generated by the camera to disk.

    try:
        client = carla.Client('localhost', configs['port'], worker_threads=1)
        client.set_timeout(10)
        world = client.load_world(configs['town'])
        world.wait_for_tick()
        # let's get the vehicle for attaching sensors
        blueprint_library = world.get_blueprint_library()
        car_blueprints = blueprint_library.filter('vehicle')
        ped_blueprints = blueprint_library.filter('walker')
        bp = random.choice(car_blueprints)
        if bp.has_attribute('color'):
            color = random.choice(bp.get_attribute('color').recommended_values)
            bp.set_attribute('color', color)
        world.wait_for_tick(3)
        transform = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(bp, transform)
        actor_list.append(vehicle)
        vehicle.set_autopilot(True)

        cameras, dest_paths = get_cameras(configs, camera_configs, vehicle, blueprint_library, world)
        actor_list.extend(cameras)

        # spawn more cars
        vehicles_list.extend(spawn_cars_new(configs, world, client))

        all_id = spawn_walkers_new(configs, world, client)
        all_walkers = world.get_actors(all_id)
        give_actors_start_and_end(world, all_id)
        offset = configs['offset']
        num_of_frames = configs['num_of_frames'] + offset
        step = configs['step']
        fps = configs['fps']
        with CarlaSyncMode(world, cameras, fps=fps) as sync_mode:
            for f in tqdm.tqdm(range(num_of_frames), total=num_of_frames, unit='it'):
                if f%step==0 and f>=offset:
                    imgs = sync_mode.tick(timeout=3.0, get_data=True)
                    for im, p in zip(imgs, dest_paths):
                        if 'depth' in p:
                            # cc = carla.ColorConverter.Raw
                            cc = get_converter['depth']
                            im.save_to_disk(p+'/%06d.png' % int(f-offset), cc)
                        elif 'semantic' in p:
                            # cc = carla.ColorConverter.CityScapesPalette
                            # cc = carla.ColorConverter.Raw
                            cc = get_converter['semantic_segmentation']
                            im.save_to_disk(p+'/%06d.png' % int(f-offset), cc)
                        else:
                            # cc = carla.ColorConverter.LogarithmicDepth
                            # cc = carla.ColorConverter.Raw
                            cc = get_converter['rgb']
                            im.save_to_disk(p+'/%06d.png' % int(f-offset), cc)
                else:
                    imgs = sync_mode.tick(timeout=3.0, get_data=False)
    finally:
        print('destroying actors')
        for actor in actor_list:
            actor.destroy()
        print('done.')
        print('\ndestroying %d vehicles' % len(vehicles_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])
        print('\stopping %d walkers' % len(all_walkers))
        for i in range(0, len(all_walkers), 2):
            all_walkers[i].stop()

if __name__ == '__main__':
    main()
