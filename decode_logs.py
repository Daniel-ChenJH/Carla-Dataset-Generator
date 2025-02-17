import carla
import random
import numpy as np
import cv2
import sys
import json
import struct
import io
import os

def read_string(file):
    length = struct.unpack('<H', file.read(2))[0]  # 读取字符串长度
    return file.read(length).decode('utf-8')  # 读取字符串

def parse_info_header(file):
    version = struct.unpack('<H', file.read(2))[0]  # 读取版本号
    magic = read_string(file)  # 读取魔法字符串
    date = struct.unpack('<I', file.read(4))[0]  # 读取时间戳
    file.read(4)    # 有四个空字节 00 00 00 00
    map_name = read_string(file)  # 读取地图名称
    return version, magic, date, map_name


def parse_packet_8(file):
    # Packet 8的格式字符串（小端）
    packet_8_format = '<Ifff?i'
    total_records, = struct.unpack('<H', file.read(2))  # 读取记录总数

    vehicle_data_list = []
    for _ in range(total_records):
        data = file.read(struct.calcsize(packet_8_format))
        vehicle_data = struct.unpack(packet_8_format, data)
        vehicle_data_list.append({
            'id': vehicle_data[0],
            'steering': vehicle_data[1],
            'throttle': vehicle_data[2],
            'brake': vehicle_data[3],
            'handbrake': vehicle_data[4],
            'gear': vehicle_data[5]
        })

    return vehicle_data_list

def parse_log_file(file_path):
    ego_vehicle_movement = {}
    frame = 0
    with open(file_path, 'rb') as file:
        # 先解析info header部分
        version, magic, date, map_name = parse_info_header(file)
        print(f"Info header: version={version}, magic='{magic}', date={date}, map_name='{map_name}'")

        # 然后解析每个packet
        while True:
            packet_header = file.read(5)  # 每个packet的header为5字节
            if not packet_header:  # 文件结束
                break

            packet_id, data_size = struct.unpack('<BI', packet_header)
            # print(packet_id, data_size)
            if packet_id == 8:
                # 读取并解析Packet 8
                packet_data = file.read(data_size)
                vehicle_data_list = parse_packet_8(io.BytesIO(packet_data))
                for vehicle_data in vehicle_data_list:
                    if int(vehicle_data['id']) == int(hero_car_id):
                        # print(f"Frame {frame}: {vehicle_data}")
                        ego_vehicle_movement[frame] = vehicle_data
            else:
                file.seek(data_size, 1)  # 跳过当前packet的数据
                if packet_id == 0: frame += 1

    f=open('ego_vehicle_behavior/'+file_path.split('/')[-1].split('.')[0]+'.json','w',encoding='utf-8')
    json.dump(ego_vehicle_movement,f,indent=4)
    return ego_vehicle_movement


if not os.path.exists('ego_vehicle_behavior'):
    os.makedirs('ego_vehicle_behavior')
client = carla.Client("localhost", 2000)
client.set_timeout(20.0)
world = client.get_world()
log_file_path = "/home/baidu/Downloads/RouteLogs"
for log in os.listdir(log_file_path):
    log_dir = os.path.join(log_file_path, log)
    log = os.path.join(log_dir, [i for i in list(os.listdir(log_dir)) if i.startswith('RouteScenario')][0])
    print('Running log: ',log)
    hero_car_info = str(client.show_recorder_file_info(log,show_all=False)).split('role_name = hero')[0].split('Create ')[-1]
    hero_car_id = hero_car_info.split(':')[0]
    print('Hero Car: ',hero_car_id,'\n',hero_car_info,'\n')
    # client.set_replayer_time_factor(2.0)
    # client.replay_file(log, 0, 0, 7544)

    parse_log_file(log)
