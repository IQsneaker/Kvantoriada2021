# coding: utf8

import rospy
from clover import srv
from std_srvs.srv import Trigger
import math
import multiprocessing
from time import sleep
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rospy.init_node('flyByPoints')
image_pub = rospy.Publisher('~view', Image)
bridge = CvBridge()

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)

path_points = './files/point.txt'
path_height = './files/height.txt'
path_wait = './files/wait.txt'
path_time = './files/time.txt'
path_coord = './files/coord.txt'
auto_arm = True
flag_wait = False
coord = []

def image_callback(data):
    latLon = get_telemetry(frame_id='body')
    lat = latLon.lat
    lon = latLon.lon

    frame = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image

    # _, frame = cap.read()
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    hsv = cv.blur(hsv, (5, 5))

    lower_white = np.array([0, 0, 171], dtype=np.uint8)
    upper_white = np.array([30, 8, 255], dtype=np.uint8)

    lower_water = np.array([0, 0, 0], dtype=np.uint8)
    upper_water = np.array([179, 55, 125], dtype=np.uint8)

    hsv = cv.erode(hsv, None, iterations=2)
    hsv = cv.dilate(hsv, None, iterations=4)

    mask_ice = cv.inRange(hsv, lower_white, upper_white)
    mask_water = cv.inRange(hsv, lower_water, upper_water)

    contours_ice, _ = cv.findContours(mask_ice, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    contours_water, _ = cv.findContours(mask_water, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

    contours_water = sorted(contours_water, key=cv.contourArea, reverse=True)

    ice_area = 0
    for i in contours_ice:
        ice_area += cv.contourArea(i)

    water_area = cv.contourArea(contours_water[0]) + ice_area

    if contours_water:
        ice_res = round(ice_area / water_area * 100)

        level = ''

        if ice_res < 30:
            level = 'Уровень заполнения льдом: Низкий'
            print(level)
        elif ice_res > 70:
            level = 'Уровень заполнения льдом: Высокий'
            print(level)
        else:
            level = 'Уровень заполнения льдом: Средний'
            print(level)

        try:
            with open(path_coord, 'a') as file:
                file.write(f'Координаты участка реки: {lat} {lon}\n{level}\n\n')
        except:
            with open(path_coord, 'w') as file:
                file.write(f'Координаты участка реки: {lat} {lon}\n{level}\n\n')

    cv.drawContours(frame, contours_ice, -1, (0, 0, 255), 3)
    cv.drawContours(frame, contours_water, 0, (0, 255, 0), 3)

    image_pub.publish(bridge.cv2_to_imgmsg(frame, 'bgr8'))

    sleep(3)

def navigate_wait(x=0, y=0, z=0, yaw=float('nan'), speed=0.5, frame_id='', auto_arm=False, tolerance=1.65):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        print(math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2))
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

def get_height():
    height = ''

    with open(path_height, 'r') as file:
        height = file.readlines()

    height = [i.replace('\n', '').split(' ') for i in height]

    if len(height[0]) != 2:
        print('\nПараметры полёта не найдены')
        exit()

    print(f'Высота - {height[0][0]} м, Скорость - {height[0][1]} м\с\n')

    verify = input('Подтвердите параметры полёта(слово yes): ')
    if verify != 'yes': exit()
    return height[0]

def get_points():
    points = ''

    with open(path_points, 'r') as file:
        points = file.readlines()

    if len(points) == 0:
        print('\nМаршрут не найден')
        exit()

    for key, i in enumerate(points):
        print(f'{key + 1}) {i}')

    verify = input('Подтвердите маршрут(слово yes): ')
    if verify != 'yes': exit()
    points = [i.replace('\n', '').split(' ') for i in points]
    return points

def navigate_global_wait(lat=0, lon=0, z=0, yaw=float('nan'), speed=1, frame_id='body', auto_arm=False, tolerance=1.65):
    global flag_wait, coord
    navigate_global(lat=lat, lon=lon, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    num = 0

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        print(math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2))
        num += 1
        if num == 10:
            num = 0
            with open(path_wait, 'r') as file:
                coordinates_wait = file.readlines()
            try:
                if coordinates_wait[0] == 'wait':
                    for i in range(12):
                        with open(path_wait, 'r') as file:
                            coordinates_wait = file.readlines()
                            coordinates_wait = [i.replace('\n', '').split(' ') for i in coordinates_wait]
                        if len(coordinates_wait[0]) == 2:
                            print('\nПараметры режима ожидания найдены')
                            navigate(frame_id='body')
                            flag_wait = True
                            coord = coordinates_wait[0]
                            break
                        sleep(10)
                    with open(path_wait, 'w') as file:
                        file.write('')
                    navigate_global(lat=lat, lon=lon, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
            except IndexError:
                pass
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

parametrs = get_height()
points = get_points()

print(parametrs)
print(points)

# Взлет
navigate_wait(z=float(parametrs[0]), frame_id='body', auto_arm=auto_arm) # , auto_arm=True

print("Good")

for key, point in enumerate(points):
    navigate_global_wait(lat=float(point[0]), lon=float(point[1]), speed=float(parametrs[1]))

    if key == 1:
        image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)

    if flag_wait:
        with open(path_wait, 'w') as file:
            file.write('')
        flag_wait = False
        navigate_global_wait(lat=float(coord[0]), lon=float(coord[1]), speed=float(parametrs[1]))
        land()
        try:
            with open(path_time, 'r') as file:
                time_wait = int(file.read())
        except:
            time_wait = 2*60
        for i in range(time_wait//10):
            with open(path_wait, 'r') as file:
                if file.read() == 'stop':
                    break
            sleep(10)
        # Взлет
        navigate_wait(z=float(parametrs[0]), frame_id='body', auto_arm=auto_arm)  # , auto_arm=True
        navigate_global_wait(lat=float(point[0]), lon=float(point[1]), speed=float(parametrs[1]))

land()
