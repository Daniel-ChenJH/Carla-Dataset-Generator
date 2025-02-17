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

import struct

def parse_frame_start(file):
    # Packet 0 - Frame Start的格式字符串（假定为小端）
    frame_start_format = '<Qdd'  # 一个64位无符号整数和两个双精度浮点数
    data = file.read(struct.calcsize(frame_start_format))
    frame_start_data = struct.unpack(frame_start_format, data)

    frame_start = {
        'id': frame_start_data[0],
        'durationThis': frame_start_data[1],
        'elapsed': frame_start_data[2]
    }

    return frame_start

def parse_event_add(file):
    total_records, = struct.unpack('<H', file.read(2))
    actors = {}
    for _ in range(total_records):
        actor_id = struct.unpack('<I', file.read(4))[0]
        actor_type = struct.unpack('<B', file.read(1))[0]
        x = struct.unpack('<f', file.read(4))[0]
        y = struct.unpack('<f', file.read(4))[0]
        z = struct.unpack('<f', file.read(4))[0]
        pitch = struct.unpack('<f', file.read(4))[0]
        yaw = struct.unpack('<f', file.read(4))[0]
        roll = struct.unpack('<f', file.read(4))[0]
        description_uid = struct.unpack('<I', file.read(4))[0]
        description_id = read_string(file)
        
        attributes_total, = struct.unpack('<H', file.read(2))
        attributes = []
        for _ in range(attributes_total):
            attr_type = struct.unpack('<B', file.read(1))[0]
            attr_id = read_string(file)
            attr_value = read_string(file)
        
        actor = {
            'location': (x, y, z),
            'rotation': (pitch, yaw, roll),
        }
        if int(actor_type)==3: actors[actor_id]=actor
    return actors

def parse_log_file(file_path):
    print('parsing...')
    ego_vehicle_movement = {}
    last_frame = 0
    with open(file_path, 'rb') as file:
        # 先解析info header部分
        version, magic, date, map_name = parse_info_header(file)
        # print(f"Info header: version={version}, magic='{magic}', date={date}, map_name='{map_name}'")

        # 然后解析每个packet
        flag = False
        traffic_lights = {}
        traffic_lights_f = {'\x01': 'YELLOW', '\x00': 'RED', '\x02': 'GREEN'}
        while True:
            
            packet_header = file.read(5)  # 每个packet的header为5字节
            if not packet_header:  # 文件结束
                break

            packet_id, data_size = struct.unpack('<BI', packet_header)
            # print(packet_id, data_size)

            if packet_id == 0: 
                d = parse_frame_start(file)
                frame = int(d['id'])
                assert frame == last_frame + 1, 'Frame should be continuous!'
                last_frame += 1
                if flag: raise ValueError('Error in parsing log file00' + frame)
                flag = True
            elif packet_id == 1:
                if not flag: raise ValueError('Error in parsing log file11' + frame)
                flag = False
                file.seek(data_size, 1)  # 跳过当前packet的数据
            elif packet_id == 2:
                if frame ==1 :
                    traffic_lights = parse_event_add(file)
                else: 
                    file.seek(data_size, 1)  # 跳过当前packet的数据
            elif packet_id == 7:
                # 读取记录总数
                total_records, = struct.unpack('<H', file.read(2))
                for _ in range(total_records):
                    # 每个交通灯的数据
                    data = file.read(10)  # 根据数据格式确定字节长度
                    if len(data) < 10:
                        break  # 文件可能结束或数据不完整
                    # 解析交通灯数据
                    database_id, is_frozen, elapsed_time, state = struct.unpack('<I?fc', data)
                    traffic_lights[database_id]['is_frozen']=bool(is_frozen)
                    traffic_lights[database_id]['state']=traffic_lights_f[state.decode('utf-8')]

            elif packet_id == 8:
                # 读取并解析Packet 8
                packet_data = file.read(data_size)
                vehicle_data_list = parse_packet_8(io.BytesIO(packet_data))
                for vehicle_data in vehicle_data_list:
                    if int(vehicle_data['id']) == int(hero_car_id):
                        # print(f"Frame {frame}: {vehicle_data}")
                        ego_vehicle_movement[frame] = vehicle_data
                        ego_vehicle_movement[frame]['traffic_lights'] = traffic_lights
            
            else:
                file.seek(data_size, 1)  # 跳过当前packet的数据

                    

    f=open('ego_vehicle_behavior/'+file_path.split('\\')[-1].split('.')[0]+'.json','w',encoding='utf-8')
    json.dump(ego_vehicle_movement,f,indent=4)
    # print('Results have been saved to ','ego_vehicle_behavior/'+file_path.split('\\')[-1].split('.')[0]+'.json')
    return ego_vehicle_movement

if not os.path.exists('ego_vehicle_behavior'):
    os.makedirs('ego_vehicle_behavior')
client = carla.Client("localhost", 2000)
client.set_timeout(20.0)
world = client.get_world()
log_file_path = os.path.join(os.getcwd(),"RouteLogs")
for log in os.listdir(log_file_path):
    if '0'!=log:continue
    log_dir = os.path.join(log_file_path, log)
    log = os.path.join(log_dir, [i for i in list(os.listdir(log_dir)) if i.startswith('RouteScenario')][0])
    print('Running log: ',log)
    # print(client.show_recorder_file_info(log,show_all=False))
    hero_car_info = str(client.show_recorder_file_info(log,show_all=False)).split('role_name = hero')[0].split('Create ')[-1]
    hero_car_id = hero_car_info.split(':')[0]
    # print('Hero Car: ',hero_car_id,'\n',hero_car_info,'\n')
    # client.set_replayer_time_factor(1.0)
    # client.replay_file(log, 0, 0, int(hero_car_id))

    parse_log_file(log)
