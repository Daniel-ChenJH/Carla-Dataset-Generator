#!/usr/bin/env python

# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
This module provides the base class for all autonomous agents
"""

from __future__ import print_function

from enum import Enum
import os
import numpy as np
import sys
import json
import carla
import cv2
import threading
from pypcd import pypcd
from queue import Queue
from srunner.scenariomanager.timer import GameTime

from leaderboard.utils.route_manipulation import downsample_route
from leaderboard.envs.sensor_interface import SensorInterface


class Track(Enum):

    """
    This enum represents the different tracks of the CARLA AD leaderboard.
    """
    SENSORS = 'SENSORS'
    MAP = 'MAP'


class AutonomousAgent(object):

    """
    Autonomous agent base class. All user agents have to be derived from this class
    """

    def __init__(self, carla_host, carla_port, debug=False):
        self.track = Track.SENSORS
        #  current global plans to reach a destination
        self._global_plan = None
        self._global_plan_world_coord = None

        # this data structure will contain all sensor data
        self.sensor_interface = SensorInterface()

        self.wallclock_t0 = None
        self.record_filepath = os.getcwd()
        bias = len(os.listdir(self.record_filepath + '/record_data'))
        self.record_filepath = self.record_filepath + '/record_data/runs' + str(bias) + '/'
        self.data_queue = Queue()
        self.record_threadings = {}
        self.record_file = {}

    
    def setup(self, path_to_conf_file):
        """
        Initialize everything needed by your agent and set the track attribute to the right type:
            Track.SENSORS : CAMERAS, LIDAR, RADAR, GPS and IMU sensors are allowed
            Track.MAP : OpenDRIVE map is also allowed
        """
        pass

    def sensors(self):  # pylint: disable=no-self-use
        """
        Define the sensor suite required by the agent

            (0, 4), (1, 5), (2, 6), (3, 7)   # 连接上下面的四条边
        ]
        self.life_time = 0.05
        
        # 绘制边界框
        for start, end in connections:
            {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                      'width': 300, 'height': 200, 'fov': 100, 'id': 'Left'},

            {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': 0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                      'width': 300, 'height': 200, 'fov': 100, 'id': 'Right'},

            {'type': 'sensor.lidar.ray_cast', 'x': 0.7, 'y': 0.0, 'z': 1.60, 'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0,
             'id': 'LIDAR'}
        ]

        """
        sensors = []

        return sensors

    def run_step(self, input_data, timestamp):
        """
        Execute one step of navigation.
        :return: control
        """
        control = carla.VehicleControl()
        control.steer = 0.0
        control.throttle = 0.0
        control.brake = 0.0
        control.hand_brake = False

        return control

    def destroy(self):
        """
        Destroy (clean-up) the agent
        :return:
        """
        # 发送结束标志给每个线程
        for queue in self.data_queue: 
            queue.put(None, None, None)

        # 等待所有线程结束
        for thread in self.record_threadings.keys():
            self.record_threadings[thread].join()

        # 清理
        self.record_threadings = {}
        self.data_queue = {}

        for f in self.record_file.keys():
            self.record_file[f].close()
        pass

    def __call__(self):
        """
        Execute the agent call, e.g. agent()
        Returns the next vehicle controls
        """
        ori_input_data = self.sensor_interface.get_data(GameTime.get_frame())

        timestamp = GameTime.get_time()
        frame_now = GameTime.get_frame()

        if not self.wallclock_t0:
            self.wallclock_t0 = GameTime.get_wallclocktime()
        wallclock = GameTime.get_wallclocktime()
        wallclock_diff = (wallclock - self.wallclock_t0).total_seconds()
        sim_ratio = 0 if wallclock_diff == 0 else timestamp/wallclock_diff

        print('=== [Agent] -- Wallclock = {} -- System time = {} -- Game time = {} -- Ratio = {}x -- Frame = {}'.format(
            str(wallclock)[:-3], format(wallclock_diff, '.3f'), format(timestamp, '.3f'), format(sim_ratio, '.3f'), frame_now))
        # print(ori_input_data.keys())
        # 确认都是同一帧数据
        assert len(np.unique(np.array([ori_input_data[i][0] for i in ori_input_data.keys()])))==1
        assert ori_input_data[list(ori_input_data.keys())[0]][0] == frame_now

        # 数据记录
        record_data_flag = False
        if record_data_flag:
            if not len(self.record_threadings.keys()):
                self.data_queue={}
                for i in ori_input_data.keys():
                    if 'camera' not in i.lower() and 'lidar' not in i.lower():
                        f = open(self.record_filepath + i + '.json', 'a+', encoding='utf-8')
                    else:
                        f = None
                    self.record_file[i] = f
                    self.data_queue[i] = Queue()
                    record_thread = threading.Thread(target=self.record_data, args = (f, i))
                    record_thread.start()
                    self.record_threadings[i] = record_thread


            gt_input_data = {'test':['test_gt_data']}
            for i in ori_input_data.keys():
                # 添加到指定的Queue
                # print(i,ori_input_data[i][1].shape)
                if True in [sensor in i.lower() for sensor in ['camera','lidar', 'speed']]:
                    self.data_queue[i].put((frame_now, i, ori_input_data[i][1]))
                    
                else:
                    self.data_queue[i].put((frame_now, i, ori_input_data[i][1].tolist()))

        # ego car针对输入信息的移动策略
        control = self.run_step(ori_input_data, timestamp)
        control.manual_gear_shift = False

        return control

    @staticmethod
    def get_ros_version():
        return -1

    def set_global_plan(self, global_plan_gps, global_plan_world_coord):
        """
        Set the plan (route) for the agent
        """
        ds_ids = downsample_route(global_plan_world_coord, 200)
        self._global_plan_world_coord = [(global_plan_world_coord[x][0], global_plan_world_coord[x][1]) for x in ds_ids]
        self._global_plan = [global_plan_gps[x] for x in ds_ids]

    def save_rgb_image(self, raw_data, filename):
        # 将原始数据从4通道转换为RGB
        rgb_data = raw_data[:, :, :3]
        cv2.imwrite(filename, rgb_data)

    def save_seg_image(self, raw_data, filename):
        cv2.imwrite(filename, raw_data)

    def reshape_seg_image(self, image):
        array = image[:, :, :3] # RGB
        # array = array[:, :, ::-1]   # BGR
        return array

    def record_data(self, f, data_type):
        # return
    
        while True:
            # 从指定的队列取数据
            (timestamp, data_type, ori_data) = self.data_queue[data_type].get()
            if timestamp == None:
                break
            
            if 'segment' in data_type.lower():
                # 语义分割相机
                # Convert and reshape image from Nx1 to shape(720, 1280, 3)
                ori_data = self.reshape_seg_image(ori_data)
                # ori_data现在是RGB格式。
                # 每个颜色代表什么物体参考https://carla.readthedocs.io/en/latest/ref_sensors/#semantic-segmentation-camera
                # 表中也为RGB格式数据
                # 总之车是深蓝色，天空是浅蓝色
                
                # 计算所有车道标记
                # Calculate all the lanes with the helper class 'LaneMarkings'
                # lanes_list, x_lanes_list = self.detect_lanemarkings(new_waypoint, image_semseg)
                
                seg_path = self.record_filepath + data_type + '/'
                if not os.path.exists(seg_path):
                    os.mkdir(seg_path)
                self.save_seg_image(ori_data, seg_path + 'Frame_' + str(timestamp) + '.jpg')

                pass
            
            elif 'camera' in data_type.lower():
                # 相机数据直接保存图片
                camera_path = self.record_filepath + data_type + '/'
                if not os.path.exists(camera_path):
                    os.mkdir(camera_path)
                self.save_rgb_image(ori_data, camera_path + 'Frame_' + str(timestamp) + '.jpg')
            elif 'lidar' in data_type.lower():
                lidar_path = self.record_filepath + data_type + '/'
                if not os.path.exists(lidar_path):
                    os.mkdir(lidar_path)

                # 使用上面获取的 x, y, z, intensity 来创建结构化数组
                dtype = [('x', 'float32'), ('y', 'float32'), ('z', 'float32'), ('intensity', 'float32')]
                structured_pc_data = np.zeros((ori_data.shape[0],), dtype=dtype)

                structured_pc_data['x'] = ori_data[:, 0]
                structured_pc_data['y'] = ori_data[:, 1]
                structured_pc_data['z'] = ori_data[:, 2]
                structured_pc_data['intensity'] = ori_data[:, 3]

                # 现在 structured_pc_data 是一个结构化数组，每个元素都包含 x, y, z, intensity 字段
                
                pc = pypcd.PointCloud.from_array(structured_pc_data)
                pc.save_pcd(lidar_path + 'Frame_' + str(timestamp) + '.pcd',compression='binary_compressed')
            else:
                # 对非图像数据才记录json文件
                json.dump({'frame':timestamp,'data':ori_data}, f)
                f.write('\n')
                f.flush()
