cameras = {}
cameras['HorizontalCameras'] = {}
cameras['HorizontalCameras']['y_locs'] = [-1.08, -0.54, 0.0, 0.54, 1.08]
cameras['HorizontalCameras']['x_locs'] = [2.92]*5
cameras['HorizontalCameras']['z_locs'] = [1.8]*5
cameras['HorizontalCameras']['rotation'] = [0, 0, 0]
cameras['HorizontalCameras']['rot_yaw'] = 0
cameras['HorizontalCameras']['sensor_types'] = ['rgb', 'semantic_segmentation', 'depth']

cameras['ForwardCameras'] = {}
cameras['ForwardCameras']['y_locs'] = [0.0]*5
cameras['ForwardCameras']['x_locs'] = [1.84, 2.38, 2.92, 3.46, 4.0]
cameras['ForwardCameras']['z_locs'] = [1.8]*5
cameras['ForwardCameras']['rotation'] = [0, 0, 0]
cameras['ForwardCameras']['rot_yaw'] = 0
cameras['ForwardCameras']['sensor_types'] = ['rgb', 'semantic_segmentation', 'depth']

cameras['SideCameras'] = {}
cameras['SideCameras']['y_locs'] = [0.0]*5
cameras['SideCameras']['x_locs']= [1.84, 2.38, 2.92, 3.46, 4.0]
cameras['SideCameras']['z_locs'] = [1.8]*5
cameras['SideCameras']['rotation'] = [0, 90.0, 0]
cameras['SideCameras']['rot_yaw'] = 90
cameras['SideCameras']['sensor_types'] = ['rgb', 'semantic_segmentation', 'depth']

configs = {}
configs['fps'] = 30 # recommended_values = >20
configs['step'] = 10 # saves every 10th frame
configs['offset'] = 100 # starts to save after the first offset frames
configs['num_of_frames'] = 1000*configs['step']
configs['num_pedestrians'] = 100
configs['number_of_vehicles'] = 100
configs['dest_path'] = '/data/teddy/temporary_carla'
