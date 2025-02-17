import carla

def calculate_3d_bounding_box(actor):
    """
    根据actor的位置、边界框extent和旋转计算在世界坐标系下的三维边界框角点。
    """
    bbox = actor.bounding_box
    actor_transform = actor.get_transform()
    
    # 边界框的角点在actor本地坐标系下的坐标
    bbox_corners = [
        carla.Vector3D(x=bbox.extent.x, y=bbox.extent.y, z=bbox.extent.z),
        carla.Vector3D(x=-bbox.extent.x, y=bbox.extent.y, z=bbox.extent.z),
        carla.Vector3D(x=-bbox.extent.x, y=-bbox.extent.y, z=bbox.extent.z),
        carla.Vector3D(x=bbox.extent.x, y=-bbox.extent.y, z=bbox.extent.z),
        carla.Vector3D(x=bbox.extent.x, y=bbox.extent.y, z=-bbox.extent.z),
        carla.Vector3D(x=-bbox.extent.x, y=bbox.extent.y, z=-bbox.extent.z),
        carla.Vector3D(x=-bbox.extent.x, y=-bbox.extent.y, z=-bbox.extent.z),
        carla.Vector3D(x=bbox.extent.x, y=-bbox.extent.y, z=-bbox.extent.z)
    ]
    
    # 将角点转换到世界坐标系
    world_corners = [actor_transform.transform(corner) for corner in bbox_corners]
    
    return world_corners

# 连接到CARLA服务器
client = carla.Client('localhost', 2000)
world = client.get_world()

# 假设已经有了ego车辆的引用
ego_vehicle = ...  # 获取ego车辆的方法依赖于你的具体场景
ego_location = ego_vehicle.get_location()

# 获取世界中的所有actors
actors = world.get_actors()

for actor in actors:
    if actor.id != ego_vehicle.id and ('vehicle' in actor.type_id or 'walker' in actor.type_id):
        distance = ego_location.distance(actor.get_location())
        if distance <= 200:  # 距离在200米以内
            world_corners = calculate_3d_bounding_box(actor, ego_location)
            print(f"Actor ID: {actor.id}, BBox Corners in World Coords: {world_corners}")
