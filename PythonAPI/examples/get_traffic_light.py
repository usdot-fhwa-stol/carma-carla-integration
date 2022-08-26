import glob
import os
import sys
try:
    sys.path.append(glob.glob('/home/yuan/CARLA_0.9.10/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg') [0])
except IndexError:
    pass
import carla
actor_list=[]
try:
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    world = client.get_world()
    map = world.get_map()
    landmark_list = map.get_all_landmarks_of_type('1000001')
    for item in landmark_list:
        print(item.id, " ", item,type)
    for landmark in landmark_list:
        world.debug.draw_string(landmark.transform.location, str(landmark.id), draw_shadow=False,
											 color=carla.Color(r=255, g=0, b=0), life_time=60000,
											 persistent_lines=True)

finally:
    print('Done!')
