import carla
import random
import numpy as np
import cv2

def process_image(image):
    """
    处理图像数据，将CARLA语义分割图像转换为RGB格式
    """
    print(image)
    image.convert(carla.ColorConverter.CityScapesPalette)  # 使用CityScapesPalette转换图像
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))  # RGBA格式
    array = array[:, :, :3]  # 取RGB部分
    array = array[:, :, ::-1]  # 将RGB转换为BGR，用于OpenCV显示
    cv2.imwrite('aaa.png', array)
    # cv2.imshow("Semantic Segmentation - CityScapes Palette", array)
    # cv2.waitKey(1)

def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(5.0)
    world = client.get_world()

    blueprint_library = world.get_blueprint_library()
    bp = blueprint_library.find('sensor.camera.semantic_segmentation')
    bp.set_attribute('image_size_x', '800')
    bp.set_attribute('image_size_y', '600')
    bp.set_attribute('fov', '90')

    spawn_point = random.choice(world.get_map().get_spawn_points())
    vehicle = world.spawn_actor(random.choice(blueprint_library.filter('vehicle.*')), spawn_point)
    # vehicle = world.spawn_actor(random.choice(blueprint_library.filter('vehicle.*')), vehicle.location)
    camera = world.spawn_actor(bp, carla.Transform(carla.Location(x=1.5, z=2.4)), attach_to=vehicle)

    camera.listen(lambda data: process_image(data))

    try:
        input("Press Enter to continue...")
    finally:
        camera.stop()
        camera.destroy()
        vehicle.destroy()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
