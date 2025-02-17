import copy
import logging
import numpy as np
import os
import time
from threading import Thread
import math
import json

from queue import Queue
from queue import Empty

import carla
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.timer import GameTime


def threaded(fn):
    def wrapper(*args, **kwargs):
        thread = Thread(target=fn, args=args, kwargs=kwargs)
        thread.setDaemon(True)
        thread.start()

        return thread
    return wrapper


class SensorConfigurationInvalid(Exception):
    """
    Exceptions thrown when the sensors used by the agent are not allowed for that specific submissions
    """

    def __init__(self, message):
        super(SensorConfigurationInvalid, self).__init__(message)


class SensorReceivedNoData(Exception):
    """
    Exceptions thrown when the sensors used by the agent take too long to receive data
    """

    def __init__(self, message):
        super(SensorReceivedNoData, self).__init__(message)


class GenericMeasurement(object):
    def __init__(self, data, frame):
        self.data = data
        self.frame = frame


class BaseReader(object):
    def __init__(self, vehicle, reading_frequency=1.0):
        self._vehicle = vehicle
        self._reading_frequency = reading_frequency
        self._callback = None
        self._run_ps = True
        # self.speed_queue = Queue()
        # self.speed_json_thread = None
        # self.speed_f = None
        self.run()

    def __call__(self):
        pass

    @threaded
    def run(self):
        first_time = True
        latest_time = GameTime.get_time()
        while self._run_ps:
            if self._callback is not None:
                current_time = GameTime.get_time()

                # Second part forces the sensors to send data at the first tick, regardless of frequency
                if current_time - latest_time > (1 / self._reading_frequency) \
                        or (first_time and GameTime.get_frame() != 0):
                    self._callback(GenericMeasurement(self.__call__(), GameTime.get_frame()))
                    latest_time = GameTime.get_time()
                    first_time = False

                else:
                    time.sleep(0.001)

    def listen(self, callback):
        # Tell that this function receives what the producer does.
        self._callback = callback

    def stop(self):
        self._run_ps = False

    def destroy(self):
        self._run_ps = False
        # if self.speed_json_thread:
        #     self.speed_queue.put(None)
        #     self.speed_json_thread.join()
        #     self.speed_json_thread = None

        # if self.speed_f:
        #     self.speed_f.close()
        #     self.speed_f = None

class SpeedometerReader(BaseReader):
    """
    Sensor to measure the speed of the vehicle.
    """
    MAX_CONNECTION_ATTEMPTS = 10

    def __init__(self, vehicle, reading_frequency, world):
        super().__init__(vehicle, reading_frequency)
        self.world = world
        self.ac=[]
        self.f=open('actors.txt','w')


    def calculate_3d_bounding_box(self, actor):
        """
        根据actor的位置、边界框extent和旋转计算在世界坐标系下的三维边界框角点。
        """
        bbox = actor.bounding_box
        actor_transform = actor.get_transform()
        
        # 边界框的角点在actor本地坐标系下的坐标
        bbox_corners = [
            carla.Vector3D(x=bbox.extent.x, y=bbox.extent.y, z=2*bbox.extent.z),
            carla.Vector3D(x=-bbox.extent.x, y=bbox.extent.y, z=2*bbox.extent.z),
            carla.Vector3D(x=-bbox.extent.x, y=-bbox.extent.y, z=2*bbox.extent.z),
            carla.Vector3D(x=bbox.extent.x, y=-bbox.extent.y, z=2*bbox.extent.z),
            carla.Vector3D(x=bbox.extent.x, y=bbox.extent.y, z=0),
            carla.Vector3D(x=-bbox.extent.x, y=bbox.extent.y, z=0),
            carla.Vector3D(x=-bbox.extent.x, y=-bbox.extent.y, z=0),
            carla.Vector3D(x=bbox.extent.x, y=-bbox.extent.y, z=0)
        ]
        
        # 将角点转换到世界坐标系
        world_corners = [actor_transform.transform(corner) for corner in bbox_corners]
        
        return [[i.x, i.y, i.z] for i in world_corners]

    def _get_forward_speed(self, transform=None, velocity=None):
        """ Convert the vehicle transform directly to forward speed """
        if not velocity:
            velocity = self._vehicle.get_velocity()
        if not transform:
            transform = self._vehicle.get_transform()

        vel_np = np.array([velocity.x, velocity.y, velocity.z])
        pitch = np.deg2rad(transform.rotation.pitch)
        yaw = np.deg2rad(transform.rotation.yaw)
        orientation = np.array([np.cos(pitch) * np.cos(yaw), np.cos(pitch) * np.sin(yaw), np.sin(pitch)])
        speed = np.dot(vel_np, orientation)

        vehicle_location = self._vehicle.get_location()

        sensor_data = [vehicle_location.x, vehicle_location.y, vehicle_location.z,
                transform.rotation.pitch, transform.rotation.yaw, transform.rotation.roll,
                orientation[0], orientation[1], orientation[2],
                velocity.x, velocity.y, velocity.z, 
                speed]
        
        return sensor_data

    def get_bbox(self):
        ego_location = self._vehicle.get_location()

        # 获取世界中的所有actors
        actors = self.world.get_actors()
        bbox_dict = {}
        check_actor_list = ['traffic', 'walker', 'vehicle', 'static.prop.warningconstruction',\
                             'static.prop.trafficwarning', 'static.prop.warningaccident', 'static.prop.dirtdebris',\
                                'static.prop.constructioncone', 'static.prop.advertisement',
                             ]
        
        for actor in actors:
            if actor.type_id not in self.ac:
                self.f.write(actor.type_id)
                self.f.write('\n')
                self.f.flush()
                self.ac.append(actor.type_id)

            if actor.id != self._vehicle.id:
                if True in [actor.type_id.startswith(i) for i in check_actor_list]:
                    actor_location = actor.get_location()
                    distance = ego_location.distance(actor_location)
                    if distance <= 200:  # 距离在200米以内
                        world_corners = self.calculate_3d_bounding_box(actor)
                        traffic_light_state = {
                            carla.TrafficLightState.Red:'-RED',
                            carla.TrafficLightState.Green:'-GREEN',
                            carla.TrafficLightState.Yellow:'-YELLOW',
                        }
                        type_id = actor.type_id  if 'traffic_light' not in actor.type_id else actor.type_id + traffic_light_state[actor.state]
                        
                        bbox_dict[actor.id] = {'type':type_id, 'coord': world_corners}
                        self.draw_bounding_box(world_corners, actor_location, type_id, str(actor.id))
                        # print(f"Actor ID: {actor.id}, BBox Corners in World Coords: {world_corners}")
        self.world.debug.draw_string(carla.Location(ego_location.x, ego_location.y-3, ego_location.z), 'ego car', draw_shadow=False, color=carla.Color(255,0,0), life_time=self.life_time)

        return bbox_dict

    def draw_bounding_box(self, bbox_corners, actor_location, label_text, actor_id):
        """
        在CARLA世界中绘制一个三维边界框和标签。
        
        :param world: CARLA世界
        :param bbox_corners: 世界坐标系下的边界框角点列表，每个元素是[x, y, z]格式
        :param label_text: 要绘制的标签文本
        :param actor_id: Actor的ID，用于生成标签文本
        """
        thickness = 0.02
        
        # 定义边界框的边，连接相应的角点
        connections = [
            (0, 1), (1, 2), (2, 3), (3, 0),  # 上面的四条边
            (4, 5), (5, 6), (6, 7), (7, 4),  # 下面的四条边
            (0, 4), (1, 5), (2, 6), (3, 7)   # 连接上下面的四条边
        ]
        self.life_time = 0.05
        
        # 绘制边界框
        # for start, end in connections:
        #     start_point = carla.Location(x=bbox_corners[start][0], y=bbox_corners[start][1], z=bbox_corners[start][2])
        #     end_point = carla.Location(x=bbox_corners[end][0], y=bbox_corners[end][1], z=bbox_corners[end][2])
        #     self.world.debug.draw_line(start_point, end_point, thickness=thickness, color=carla.Color(0,0,150,128), life_time=self.life_time+0.0001)

        # 计算标签的位置  标签只有在spectator 才能看到，在camera里面看不到
        # spectator 视角中 上方为x正方向 右侧为y正方向
        label_location = carla.Location(actor_location.x, actor_location.y-3, actor_location.z)
        text = label_text if 'walker' in label_text else label_text.split('.',1)[1] if 'traffic' in label_text else label_text.split('.')[0]
        # 绘制标签
        self.world.debug.draw_string(label_location, text, draw_shadow=False, color=carla.Color(100,100,255,128), life_time=self.life_time)


    def to_json(self):
        while True:
            data = self.speed_queue.get()
            if data is None:
                break
            timestamp = GameTime.get_frame()
            json.dump({'timestamp':timestamp,'data':data}, self.speed_f)
            self.speed_f.write('\n')
            self.speed_f.flush()


    def __call__(self):
        """ We convert the vehicle physics information into a convenient dictionary """

        # protect this access against timeout
        attempts = 0
        while attempts < self.MAX_CONNECTION_ATTEMPTS:
            try:
                velocity = self._vehicle.get_velocity()
                transform = self._vehicle.get_transform()
                break
            except Exception:
                attempts += 1
                time.sleep(0.2)
                continue

        return {'ego_car_info': self._get_forward_speed(transform=transform, velocity=velocity),
                'bbox': self.get_bbox(),
                }


class OpenDriveMapReader(BaseReader):
    def __call__(self):
        return {'opendrive': CarlaDataProvider.get_map().to_opendrive()}


class CallBack(object):
    def __init__(self, tag, sensor_type, sensor, data_provider):
        self._tag = tag
        self._data_provider = data_provider

        self._data_provider.register_sensor(tag, sensor_type, sensor)

    def __call__(self, data):
        if isinstance(data, carla.libcarla.Image):
            self._parse_image_cb(data, self._tag)
        elif isinstance(data, carla.libcarla.LidarMeasurement):
            self._parse_lidar_cb(data, self._tag)
        elif isinstance(data, carla.libcarla.RadarMeasurement):
            self._parse_radar_cb(data, self._tag)
        elif isinstance(data, carla.libcarla.GnssMeasurement):
            self._parse_gnss_cb(data, self._tag)
        elif isinstance(data, carla.libcarla.IMUMeasurement):
            self._parse_imu_cb(data, self._tag)
        elif isinstance(data, GenericMeasurement):
            self._parse_pseudosensor(data, self._tag)
        else:
            logging.error('No callback method for this sensor.')

    # Parsing CARLA physical Sensors
    def _parse_image_cb(self, image, tag):
        if 'segment' in tag.lower():
            image.convert(carla.ColorConverter.CityScapesPalette)  # 使用CityScapesPalette转换图像
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = copy.deepcopy(array)
        array = np.reshape(array, (image.height, image.width, 4))
        self._data_provider.update_sensor(tag, array, image.frame)

    def _parse_lidar_cb(self, lidar_data, tag):
        points = np.frombuffer(lidar_data.raw_data, dtype=np.dtype('f4'))
        points = copy.deepcopy(points)
        points = np.reshape(points, (int(points.shape[0] / 4), 4))
        self._data_provider.update_sensor(tag, points, lidar_data.frame)

    def _parse_radar_cb(self, radar_data, tag):
        # [depth, azimuth, altitute, velocity]
        points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
        points = copy.deepcopy(points)
        points = np.reshape(points, (int(points.shape[0] / 4), 4))
        points = np.flip(points, 1)
        self._data_provider.update_sensor(tag, points, radar_data.frame)

    def _parse_gnss_cb(self, gnss_data, tag):
        array = np.array([gnss_data.latitude,
                          gnss_data.longitude,
                          gnss_data.altitude], dtype=np.float64)
        self._data_provider.update_sensor(tag, array, gnss_data.frame)

    def _parse_imu_cb(self, imu_data, tag):
        array = np.array([imu_data.accelerometer.x,
                          imu_data.accelerometer.y,
                          imu_data.accelerometer.z,
                          imu_data.gyroscope.x,
                          imu_data.gyroscope.y,
                          imu_data.gyroscope.z,
                          imu_data.compass,
                         ], dtype=np.float64)
        self._data_provider.update_sensor(tag, array, imu_data.frame)

    def _parse_pseudosensor(self, package, tag):
        self._data_provider.update_sensor(tag, package.data, package.frame)


class SensorInterface(object):
    def __init__(self):
        self._sensors_objects = {}
        self._data_buffers = Queue()
        self._queue_timeout = 10

        # Only sensor that doesn't get the data on tick, needs special treatment
        self._opendrive_tag = None

    def register_sensor(self, tag, sensor_type, sensor):
        if tag in self._sensors_objects:
            raise SensorConfigurationInvalid("Duplicated sensor tag [{}]".format(tag))

        self._sensors_objects[tag] = sensor

        if sensor_type == 'sensor.opendrive_map': 
            self._opendrive_tag = tag

    def update_sensor(self, tag, data, frame):
        if tag not in self._sensors_objects:
            raise SensorConfigurationInvalid("The sensor with tag [{}] has not been created!".format(tag))

        self._data_buffers.put((tag, frame, data))

    def get_data(self, frame):
        """Read the queue to get the sensors data"""
        # 这里返回车辆传感器读取的到的原始信息
        try:
            data_dict = {}
            while len(data_dict.keys()) < len(self._sensors_objects.keys()):
                # Don't wait for the opendrive sensor
                if self._opendrive_tag and self._opendrive_tag not in data_dict.keys() \
                        and len(self._sensors_objects.keys()) == len(data_dict.keys()) + 1:
                    break

                sensor_data = self._data_buffers.get(True, self._queue_timeout)
                if sensor_data[1] != frame:
                    continue
                # if 'SPEEDOMETER' == sensor_data[0]:
                #     data_dict['SPEEDOMETER'] = ((sensor_data[1], sensor_data[2]['ego_car_info']))
                #     data_dict['BBOX'] = ((sensor_data[1], sensor_data[2]['bbox']))
                # else:
                data_dict[sensor_data[0]] = ((sensor_data[1], sensor_data[2]))
                
        except Empty:
            raise SensorReceivedNoData("A sensor took too long to send their data")

        return data_dict
