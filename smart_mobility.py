#!/usr/bin/env python
# -*- coding: utf-8 -*-

# import modules and messages
from turtle import speed
import rospy, rospkg
import numpy as np
import cv2, random, math, time
import sys
import os
import signal

# AR tag
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion

# Lidar
from sensor_msgs.msg import LaserScan

# Motor
from xycar_motor.msg import xycar_motor

# Open CV
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Traffic Light
from std_msgs.msg import String
from std_msgs.msg import Int64

# lidar distance
distance = []

# ARtag data
arData = [{"DX":0.0,"DY":0.0,"DZ":0.0,"AX":0.0,"AY":0.0,"AZ":0.0,"AW":0.0, "ID":-1}, {"DX":0.0,"DY":0.0,"DZ":0.0,"AX":0.0,"AY":0.0,"AZ":0.0,"AW":0.0, "ID":-1}] # AR 태그에서 사용

# image and motor
msg = xycar_motor() 
bridge = CvBridge() 
image = np.empty(shape=[0])


##########################################################################################################
####################################Mission Variables#####################################################
##########################################################################################################


# Variables for line tracking
pub = None
Width = 640
Height = 480
Offset = 320
Gap = 40
center_gap = 30 # 차선 인식 제외구간
drive_error = 0 # 카메라 위치에 따른 displacement 조정용


# one_line drive
distance_center_r = 150 # distance from center, can be minus or 0
distance_center_l = 210 # distance from center, can be minus or 0


# Variables for obstacle detour
left_min = 130 # degree
left_max = 150
right_min = 230
right_max = 250

detour_time1 = 18 # x/10sec to detour
detour_time2 = 30 # x/10sec to detour
straight_time =22 # x/10sec to go straight
corner_time =28 # x/10sec to turn
obstacle_distance = 0.45 # obstacle detect range
obstacle_num = 1 # detour if obstacle num is over x


# Variables for stop line detect
stop_limit = 2 # sec limit for red light
threshold_100 = 165 # binary threshold
lbound = np.array([0, 0, threshold_100], dtype=np.uint8) # binary array
ubound = np.array([131, 255, 255], dtype=np.uint8)

width_640 = 640 # scan range
scan_width_200, scan_height_20 = 320, 20 
area_width_20, area_height_10 = 20, 10
vertical_430 = 400
row_begin_5 = (scan_height_20 - area_height_10) // 2
row_end_15 = row_begin_5 + area_height_10
nonzero_limit = 150 # stop line threshold
stop_square_limit = 10 # square limit to detect stopline


# slow mission
slow_done = 0 # 1 if slow mission is done


# Variables for cross street
time_crossing = 7 # sec to pass crosswalk


# Variables for parking
back_time = 14 # x/10 sec to go back
back_angle = 35 # back drive angle
angle_coefficient = 5 # base on DX



####Init value####

# mode
mode = 1
trigger = 0

# xycar angle and speed
Angle = 0
Speed = 5

# time record
start_time = 0
end_time = 0


def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


# subscriber callback functions
def img_callback(img_data):
    global image  
    global bridge  
    image = bridge.imgmsg_to_cv2(img_data, "bgr8")


def lidar_callback(data):
    global distance
    global msg
    distance = data.ranges

def AR_callback(data):
    global arData
    tag_index = 0
    for i in data.markers:
        arData[tag_index]["DX"] = i.pose.pose.position.x
        arData[tag_index]["DY"] = i.pose.pose.position.y
        arData[tag_index]["DZ"] = i.pose.pose.position.z
        arData[tag_index]["AX"] = i.pose.pose.orientation.x
        arData[tag_index]["AY"] = i.pose.pose.orientation.y
        arData[tag_index]["AZ"] = i.pose.pose.orientation.z
        arData[tag_index]["AW"] = i.pose.pose.orientation.w
        arData[tag_index]["ID"] = i.id
        tag_index += 1

def Traffic_callback_L(data):
    global light_color_L
    light_color_L = data.data

def Traffic_callback_R(data):
    global light_color_R
    light_color_R = data.data

def Traffic_callback_C(data):
    global light_count
    light_count = data.data


# publish xycar_motor msg
def drive(Angle, Speed): 
    global pub
    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed
    pub.publish(msg)


# draw lines
def draw_lines(img, lines):
    global Offset
    for line in lines:
        x1, y1, x2, y2 = line[0]
        color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        img = cv2.line(img, (x1, y1+Offset), (x2, y2+Offset), color, 2)
    return img


# draw rectangle
def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) / 2

    cv2.rectangle(img, (lpos - 5, 15 + offset),
                    (lpos + 5, 25 + offset),
                    (0, 255, 0), 2)
    cv2.rectangle(img, (rpos - 5, 15 + offset),
                    (rpos + 5, 25 + offset),
                    (0, 255, 0), 2)
    cv2.rectangle(img, (center-5, 15 + offset),
                    (center+5, 25 + offset),
                    (0, 255, 0), 2)
    cv2.rectangle(img, (315, 15 + offset),
                    (325, 25 + offset),
                    (0, 0, 255), 2)
    return img


# left lines, right lines
def divide_left_right(lines):
    global Width

    low_slope_threshold = 0
    high_slope_threshold = 10


    # calculate slope & filtering with threshold
    slopes = []
    new_lines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]

        if x2 - x1 == 0:
            slope = 0
        else:
            slope = float(y2-y1) / float(x2-x1)

        if abs(slope) > low_slope_threshold and abs(slope) < high_slope_threshold:
            slopes.append(slope)
            new_lines.append(line[0])


    # divide lines left to right
    left_lines = []
    right_lines = []

    for j in range(len(slopes)):
        Line = new_lines[j]
        slope = slopes[j]

        x1, y1, x2, y2 = Line

        if (slope < 0) and (x2 < Width/2 - center_gap):
            left_lines.append([Line.tolist()])
        elif (slope > 0) and (x1 > Width/2 + center_gap):
            right_lines.append([Line.tolist()])

    return left_lines, right_lines


# get average m, b of lines
def get_line_params(lines):
    # sum of x, y, m
    x_sum = 0.0
    y_sum = 0.0
    m_sum = 0.0

    size = len(lines)
    if size == 0:
        return 0, 0

    for line in lines:
        x1, y1, x2, y2 = line[0]
        x_sum += x1 + x2
        y_sum += y1 + y2
        m_sum += float(y2 - y1) / float(x2 - x1)

    x_avg = x_sum / (size * 2)
    y_avg = y_sum / (size * 2)
    m = m_sum / size
    b = y_avg - m * x_avg

    return m, b

# get lpos, rpos
def get_line_pos(img, lines, left=False, right=False): # 좌or우 line 받아와 pos return
    global Width, Height
    global Offset, Gap

    m, b = get_line_params(lines)

    if m == 0 and b == 0: # line out
        if left:
            pos = 0
        if right:
            pos = Width
    else:
        y = Gap / 2
        pos = (y - b) / m

        b += Offset
        x1 = (Height - b) / float(m)
        x2 = ((Height/2) - b) / float(m)

        cv2.line(img, (int(x1), Height), (int(x2), (Height/2)), (255, 0,0), 3)

    return img, int(pos)

# show image and return lpos, rpos
def process_image(frame):
    global Width
    global Offset, Gap

    # grayimg_callback
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    # blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)

    # canny edge
    low_threshold = 60 # non-edge 상한
    high_threshold = 70 # edge 하한
    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)

    # HoughLinesP
    roi = edge_img[Offset : Offset+Gap, 0 : Width]
    all_lines = cv2.HoughLinesP(roi,1,math.pi/180,30,30,10) # threshold, minlinelength, maxlinegap

    # divide left, right lines
    if all_lines is None:
        return 0, 640
    left_lines, right_lines = divide_left_right(all_lines)

    # get center of lines
    frame, lpos = get_line_pos(frame, left_lines, left=True)
    frame, rpos = get_line_pos(frame, right_lines, right=True)

    # draw lines
    frame = draw_lines(frame, left_lines)
    frame = draw_lines(frame, right_lines)
    frame = cv2.line(frame, (230, 235), (410, 235), (255,255,255), 2)
                                
    # draw rectangle
    frame = draw_rectangle(frame, lpos, rpos, offset=Offset)


    # show image
    cv2.imshow('calibration', frame)

    return lpos, rpos


def hough_drive():
    global Angle, Speed, Width, Height, image, drive_error
    
    lpos, rpos = process_image(image) #left and right coordinates return

    center = (lpos + rpos) / 2 - drive_error

    if center != 320:
        Angle = -(Width/2 - center)  # angle

    
    drive(Angle, Speed)


# 평소 hough drive, 오른쪽 차선 잃었을 시 왼쪽 차선만으로 주행
def hough_drive_sudo_left():
    global Angle, Speed, Width, Height, image, cap
    
    lpos, rpos = process_image(image)
    
    center = (lpos + rpos) / 2 - drive_error

    # 어느 한 차선을 잃지 않았을 시
    if center != 320:
        # 오른쪽 차선을 잃었다면 왼쪽 차선만으로 주행
        if rpos == 640:
            Angle = -(195-lpos)

        # 두 차선 모두 있을 시 hough drive
        else:
            Angle = -(Width/2 - center)

    drive(Angle, Speed)


# 평소 hough drive, 왼쪽 차선 잃었을 시 오른쪽 차선만으로 주행
def hough_drive_sudo_right():
    global Angle, Speed, Width, Height, image, cap
    
    lpos, rpos = process_image(image)
    
    center = (lpos + rpos) / 2 - drive_error

    # 어느 한 차선을 잃지 않았을 시
    if center != 320:
        # 왼쪽 차선을 잃었다면 오른쪽 차선만으로 주행
        if lpos == 0:
            Angle = -(640-140-rpos)

        # 두 차선 모두 있을 시 hough drive
        else:
            Angle = -(Width/2 - center)

    drive(Angle, Speed)


# 왼쪽 차선만으로 왼쪽 차선에 붙어서 주행
def hough_drive_left():
    global Angle, Speed,  Width, Height, image, cap
    
    lpos, rpos = process_image(image)

    # 왼쪽 차선을 잃지 않았을 시 왼쪽 차선만으로 주행
    if lpos != 0:
        Angle = -(distance_center_l-lpos)

    drive(Angle, Speed)


# 오른쪽 차선만으로 왼쪽 차선에 붙어서 주행
def hough_drive_left2():
    global Angle, Speed,  Width, Height, image, cap
    
    lpos, rpos = process_image(image)

    # 오른쪽 차선을 잃지 않았을 시 오른쪽 차선만으로 주행
    if rpos != 640:
        Angle = -(640-distance_center_r+120-rpos)

    drive(Angle, Speed)


# 오른쪽 차선만으로  오른쪽 차선에 붙어서 주행
def hough_drive_right():
    global mode, Angle, Speed,  Width, Height, image, cap
    
    lpos, rpos = process_image(image)

    # 오른쪽 차선을 잃지 않았을 시 오른쪽 차선만으로 주행
    if rpos != 640:
        Angle = -(640-distance_center_r-rpos)

    drive(Angle, Speed)


# 미션의 시작을 체크하여 미션 트리거를 활성화
def mission_check():

    global distance, mode, trigger, image, Speed, start_time, end_time, slow_done



    # 모드 1 (장애물 회피)
    if mode == 1:
        i = 0
        # right obstacle check, trigger = 1
        for degree in range(right_min,right_max): # 오른쪽 범위 확인
            if distance[degree] <= obstacle_distance:
                print(degree)
                i += 1
            
            if i > obstacle_num:
                trigger = 1
                print("right obstacle check!!!!")
                return
    
        i = 0
        # left obstacle check, trigger = 2
        for degree in range(left_min,left_max): # 왼쪽 범위 확인
            if distance[degree] <= obstacle_distance:
                print(degree)
                i += 1
            
            if i > obstacle_num:
                trigger = 2
                print("left obstacle check!!!!")
                return



    # 모드 2 (정지선 감지)
    if mode == 2:

        # 이미지 로드와 이진화
        while not image.size == (640*480*3):
            continue
        frame = image
        roi = frame[vertical_430:vertical_430 + scan_height_20, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        bin = cv2.inRange(hsv, lbound, ubound)

        # 이진화된 이미지에서 정지선 체크
        stop_line = 0
        for l in range(290+3*Angle, 350+3*Angle, 1):
            area = bin[row_begin_5:row_end_15, l - (area_width_20/2):l + (area_width_20/2)] 
            if cv2.countNonZero(area) > nonzero_limit:
                stop_line += 1

        # 정지선 감지 시 trigger=1
        if stop_line > stop_square_limit : 
            trigger = 1
            print("Stopline check!!!!!!")
            return



    # 모드 3 (갈림길 감지)
    if mode == 3:

        # AR태그의 ID와 거리 감지, 감지 시 trigger = 1
        if (arData[0]["ID"] == 2 and arData[0]["DZ"] <= 0.7):
            print("Cross AR CHECK!!!!")
            trigger = 1

        elif (arData[1]["ID"] == 2 and arData[1]["DZ"] <= 0.7):
            print("Cross AR CHECK!!!!")
            trigger = 1



    # 모드 4 (정지신호 감지)
    if mode == 4:

        # AR태그의 ID와 거리 감지, 감지 시 trigger = 1
        if (arData[0]["ID"] == 4 and arData[0]["DZ"] <= 0.5):
            print("STOP AR CHECK!!!!")
            trigger = 1

        elif (arData[1]["ID"] == 4 and arData[1]["DZ"] <= 0.5):
            print("STOP AR CHECK!!!!")
            trigger = 1



    # Slow mission (AR태그의 ID와 거리, slow mission의 실행 여부 감지)
    if slow_done == 0 and (mode == 3)  and  (((arData[0]["ID"] == 0) and (arData[0]["DZ"] < 0.9)) or ((arData[1]["ID"] == 0) and (arData[1]["DZ"] < 0.9))):

        print('Slow tag detect!!!!!!!!!')

        # slow mission 시작시간 저장 및 감속
        start_time = time.time()
        Speed = 3
        slow_done = 1

    # 감속으로부터 10초 이상 지났을 시 속도 5로 가속
    if (start_time != 0) and (time.time() - start_time) > 10:
        Speed = 5
        start_time = 0
        print("Speed up!!!!!!!")



# mission1 (장애물회피주행)
def mission1():
    global distance, Speed, mode, trigger
    print("Mission 1 Start!!!")

    # right obstacle detect
    if trigger == 1:

        # detour_time1 동안 왼쪽 차선에 붙어 주행
        for i in range(detour_time1):
            hough_drive_left()
            time.sleep(0.1)

    # left obstacle detect
    else:

        # detour_time1 동안 오른쪽 차선에 붙어 주행
        for i in range(detour_time1):
            hough_drive_right()
            time.sleep(0.1)


    # 2번 트리거의 두 번째 장애물
    if trigger == 2:

        # detour_time2 동안 왼쪽 차선에 붙어 주행
        for i in range(detour_time2):
            hough_drive_left()
            time.sleep(0.1)

    # 1번 트리거의 두 번째 장애물
    else:

        # detour_time2 동안 오른쪽 차선에 붙어 주행
        for i in range(detour_time2):
            hough_drive_right()
            time.sleep(0.1)

    # 장애물 회피 후 직선주행
    for i in range(30):
        hough_drive()
        time.sleep(0.1)

    # 코너 확인
    corner_count = 0
    for i in range(straight_time):
        hough_drive()
        time.sleep(0.1)

        if Angle < -3:
            corner_count += 1
        elif Angle > 3:
            corner_count -= 1

        if corner_count > 2:
            break

    # 코너 주행
    for i in range(corner_time):
        drive(-50,3)
        time.sleep(0.1)

    # 코너 주행 후 mode 변경, trigger 초기화
    mode = 2
    trigger = 0



# mission2 (신호등 감지 및 주행)
def mission2():
    global distance, Speed, mode, trigger

    print('Mission2 start!!!!!!')
    
    # 해당 루프를 반복
    while True:

        # 초록불이거나 통과 가능한 노란불일 시
        if (light_color_L) == 'G' or (light_color_L == 'Y' and light_count > stop_limit):

            # hough_drive 주행
            print("G/Y light detect!!!!")
            for i in range(40):
                hough_drive()
                time.sleep(0.1)

            # 코너 확인
            corner_count = 0
            while True:
                hough_drive()
                time.sleep(0.1)

                if Angle < -5:
                    corner_count += 1
                elif Angle > 0:
                    corner_count -= 1

                if corner_count > 5:
                    break

            # 코너 주행
            for i in range(corner_time-17):
                drive(-50,5)
                time.sleep(0.1)

            # 코너 주행 후 mode 변경, trigger 초기화
            mode = 3
            trigger = 0
            return

        # 빨간불이거나 통과 불가능한 노란불일 시
        else:
            print("Stop light detect!!!!")

            # 정지
            drive(0,0)
            time.sleep(0.1)

# mission3 (갈림길 주행)
def mission3():
    global distance, Speed, mode, trigger

    print("Mission3 start!!!!!!!!!!")

    time_left = time_crossing
    time_right = time_crossing

    # 왼쪽 신호등이 초록불이거나 노란불일 시
    if (light_color_L == 'G') or (light_color_L == 'Y'):

        # 신호등에 도달하는 시간보다 남아있는 시간이 길다면 time_left = 0
        if (light_count > time_crossing):
            time_left = 0

        # 신호등에 도달하는 시간보다 남아있는 시간이 짧다면 다음 초록불까지 걸리는 시간을 time_left에 저장
        else:
            time_left = 10 + light_count - time_crossing

        # 왼쪽 신호등을 바로 통과할 수 없다면 time_right = 0
        if (time_left != 0):
            time_right = 0

        # 왼쪽 신호등을 바로 통과할 수 있다면 time_right = 10
        else:
            time_right = 10

    # 왼쪽 신호등이 빨간불일 시
    else:

        # 신호등에 도달하는 시간보다 남아있는 시간이 길다면 time_right = 0
        if (light_count > time_crossing):
            time_right = 0

        #신호등에 도달하는 시간보다 남아있는 시간이 짧다면 다음 초록불까지 걸리는 시간을 time_right에 저장
        else:
            time_right = 10 + light_count - time_crossing

        # 오른쪽 신호등을 바로 통과할 수 없다면 time_left = 0
        if (time_right != 0):
            time_left = 0

        # 오른쪽 신호등을 바로 통과할 수 있다면 time_left = 10
        else:
            time_left = 10


    # 왼쪽 길이 더 빠를 시
    if (time_left < time_right):
        for i in range(27):
            hough_drive_right()
            time.sleep(0.1)
    
        for i in range(16):
            drive(-35, 5)
            time.sleep(0.1)

        for i in range(10):
            drive(50, 5)
            time.sleep(0.1)

    # 오른쪽 길이 더 빠를 시
    else:
        for i in range(31):
            hough_drive_left2()
            time.sleep(0.1)

        for i in range(11):
            drive(40, 5)
            time.sleep(0.1)

        for i in range(10):
            drive(-50, 5)
            time.sleep(0.1)


    stopline = 0

    # 주어진 코드를 반복
    for i in range (20):

        if stopline == 0:

            # 이미지 로드와 이진화
            while not image.size == (640*480*3):
                continue
            frame = image
            roi = frame[vertical_430:vertical_430 + scan_height_20, :]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            bin = cv2.inRange(hsv, lbound, ubound)

            # 이진화된 이미지에서 정지선 감지
            stop_count = 0
            for l in range(290+3*Angle, 350+3*Angle, 1):
                area = bin[row_begin_5:row_end_15, l - (area_width_20/2):l + (area_width_20/2)] 
                if cv2.countNonZero(area) >  (nonzero_limit-50):
                    stop_count += 1

            # 정지선 감지 시 stopline = 1
            if stop_count > (stop_square_limit-8) :
                        
                print("Stopline check!!!!!!")
                stopline = 1


        # 왼쪽 길을 선택했을 시
        if (time_left <= time_right):

            # 정지선 감지 시
            if (stopline == 1):

                # 빨간불이거나 통과 불가능한 노란불일 시
                if ((light_color_L == 'R')or(light_color_L == 'Y' and light_count <= stop_limit)):
                    print("Red stop")

                    # 초록불이 켜질때까지 정지
                    while (light_color_L != 'G'):
                        drive(0,0)
                        time.sleep(0.1)
                
                # 정지선 통과 시 stopline = 2
                stopline = 2


            # 정지선 통과 시
            if (stopline == 2):

                # hough_drive_sudo_left 함수를 통해 주행
                for i in range(60):
                    hough_drive_sudo_left()
                    time.sleep(0.1)

                # 주행 완료 후 stopline = 3
                stopline = 3

            else:
                hough_drive()

        

        # 오른쪽 길을 선택했을 시
        else :

            # 정지선 감지 시
            if (stopline == 1):

                # 빨간불이거나 통과 불가능한 노란불일 시
                if ((light_color_R == 'R')or(light_color_R == 'Y' and light_count <= stop_limit)):
                    print("Red stop")

                    # 초록불이 켜질때까지 정지
                    while (light_color_R != 'G'):
                        drive(0,0)
                        time.sleep(0.1)

                # 정지선 통과 시 stopline = 2
                stopline = 2

            # 정지선 통과 시
            if (stopline == 2):

                # hough_drive_sudo_right 함수를 통해 주행
                for i in range(60):
                    hough_drive_sudo_right()
                    time.sleep(0.1)

                # 주행 완료 후 stopline = 3
                stopline = 3

            else:
                hough_drive()


        time.sleep(0.1)


    # 갈림길 종료 후 mode 변경, trigger 초기화
    mode = 4
    trigger = 0



# mission4 (주차 수행)
def mission4():
    global Angle, distance, Speed, mode, trigger

    print("Mission4 start!!!!!!!!!!")

    drive(back_angle, 0)
    time.sleep(0.2)

    # 미션 시작 시의 AR태그와의 거리 계산
    start_point = arData[0]["DZ"]

    # 모터 구동 위한 루프
    while True:

        drive(0, -3)

        # 미션 시작 시의 거리보다 멀어졌을 시 루프 종료
        if (start_point + 0.02 < arData[0]["DZ"]):
            break

        time.sleep(0.1)

    # back_time 동안 후진주행
    for i in range(back_time):
        drive(back_angle, -5)
        time.sleep(0.1)

    # back_time+1 동안 후진주행
    for i in range(back_time+1):
        drive(-back_angle, -5)
        time.sleep(0.1)


    # 주차 루프
    while True:

        # 주차 AR태그와의 거리가 0.7 이상일 시
        if (arData[0]["ID"] == 6) and (arData[0]["DZ"] >= 0.7):

            # AR태그방향으로 조향
            Angle = (arData[0]["DX"])*angle_coefficient


        # 주차 AR태그와의 거리가 0.7 이상일 시
        elif (arData[1]["ID"] == 6) and (arData[1]["DZ"] >= 0.7):

            # AR태그방향으로 조향
            Angle = (arData[1]["DX"])*angle_coefficient


        # 주차 AR태그와의 거리가 0.7 이하일 시
        elif ((arData[0]["ID"] == 6) and (arData[0]["DZ"] <= 0.7)) or ((arData[1]["ID"] == 6) and (arData[1]["DZ"] <= 0.7)):
            
            print("PARK STOP")

            # 운행 정지
            while True:
                drive(0,0)
                time.sleep(0.1)


        drive(Angle, Speed)

        time.sleep(0.1)




def start():
    global pub
    global image
    global cap
    global Width, Height
    global mode
    global Speed

    rospy.init_node('auto_drive')
    rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    rospy.Subscriber('/scan', LaserScan, lidar_callback, queue_size = 1)
    rospy.Subscriber('ar_pose_marker', AlvarMarkers, AR_callback)
    rospy.Subscriber('/Left_color', String, Traffic_callback_L, queue_size = 1)
    rospy.Subscriber('/Right_color', String, Traffic_callback_R, queue_size = 1)
    rospy.Subscriber('/time_count', Int64, Traffic_callback_C, queue_size = 1)



    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    print ("---------- Xycar A2 v1.0 ----------")
    rospy.sleep(2)

    while not rospy.is_shutdown():
        while not image.size == (640*480*3):
            continue
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # check mission and active trigger
        mission_check()

        # normal drive
        if trigger == 0:
            hough_drive()

        # obstacle detect
        elif (mode == 1):
            Speed = 3
            mission1()

        # stopline detect
        elif (mode == 2):
            Speed = 5
            mission2()

        # cross ARtag detect
        elif (mode == 3):
            mission3()

        # park ARtag detect
        elif (mode == 4):
            mission4()
    
    rospy.spin()

if __name__ == '__main__':

    start()

