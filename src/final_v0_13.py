#!/usr/bin/env python

# motor mapping 0.8 if angle is between 20 and 30

import cv2, rospy, rospkg, sys, os, signal, math
import numpy as np
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)

# all global variables
tm = cv2.TickMeter()
Gap = 70
Offset = 0
# center offset of xycar in usb camera
center_offset = 20
# PID values
ie, de, pe = 0, 0, 0
# image shape
Height, Width = 480, 640
# bridge for changing /usb_cam/image_raw topic to OpenCV image type
bridge = CvBridge()

pub = None

l_ex, r_ex = 0, 0

# Set ROI points for bird eye view
src_pts = np.array([[70,330],[500,330],[600,400],[0,400]],dtype=np.float32)
dst_pts = np.array([[0,0],[639,0],[639,479],[0,479]],dtype=np.float32)
mat_pers = cv2.getPerspectiveTransform(src_pts, dst_pts)
# For inRange function but this will not be used for now (22.4.5)
lbound = np.array([0,0,0], dtype=np.uint8)
rbound = np.array([75,255,255], dtype=np.uint8)

# calibrating for usb_camera.
calibrated = True
if calibrated:
    mtx = np.array([
        [422.037858, 0.0, 245.895397],
        [0.0, 435.589734, 163.625535],
        [0.0, 0.0, 1.0]
    ])
    dist = np.array([-0.289296, 0.061035, 0.001786, 0.015238, 0.0])
    cal_mtx, cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (Width, Height), 1, (Width, Height))

def calibrate_image(frame,Height,Width):
    global mtx, dist
    global cal_mtx, cal_roi
    tf_image = cv2.undistort(frame, mtx, dist, None, cal_mtx)
    x, y, w, h = cal_roi
    tf_image = tf_image[y:y+h, x:x+w]
    return cv2.resize(tf_image, (Width, Height))

# callback function for Subscriber node
def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

# publish xycar_motor msg
def drive(Angle, Speed):
    global pub
    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed
    pub.publish(msg)

# dividing left, right lines with HoughLinesP result
def divide_left_right(lines):
    global Width
    low_slope_threshold = 0
    high_slope_threshold = 1000
    # calculate slope & filtering withcenter threshold
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

        if (slope < 0) and (x1 < Width/2):
            left_lines.append([Line.tolist()])
        elif (slope > 0) and (x2 > Width/2):
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

def get_line_pos(img, lines, left=False, right=False):
    global Width, Height
    global Offset, Gap

    m, b = get_line_params(lines)

    if m == 0 and b == 0:
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

        cv2.line(img, (int(x1), Height), (int(x2), (Height/2)), 0, 3)

    return img, int(pos)

def process_image(frame):
    global Height, Width, mat_pers
    # gray
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # calibrate
    gray_cal = calibrate_image(gray, Height, Width)
    # ROI selection (bird eye view)
    roi = cv2.warpPerspective(gray_cal, mat_pers, (Width,Height))
    # blur
    roi = cv2.GaussianBlur(roi, (0,0), 3)
    # Canny
    roi_canny = cv2.Canny(roi, 40, 50)
    # HoughLinesP
    lines = cv2.HoughLinesP(roi_canny, 1.0, math.pi/180, 30, minLineLength=40, maxLineGap=10)
    # divide_left_right
    if lines is None:
        return l_ex, r_ex
    left_lines, right_lines = divide_left_right(lines)

    # get center of lines
    frame, lpos = get_line_pos(roi, left_lines, left=True)
    frame, rpos = get_line_pos(roi, right_lines, right=True)
    if (lpos ==0 and rpos ==640):
        return l_ex, r_ex
    elif (lpos ==0):
        lpos = rpos-500
    elif (rpos ==640):
        rpos = lpos+500
    return lpos, rpos

def PID(error,p_gain,i_gain,d_gain):
    global ie, de, pe
    de = error - pe
    pe = error
    if (-3500< ie < 3500):
        ie += error
    else:
        ie = 0

    return p_gain*pe + i_gain*ie + d_gain*de

def start():
    global pub
    global image
    global cap
    global Width, Height
    global tm

    rospy.init_node('auto_drive')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    print "---------- Xycar A2 v1.0 ----------"
    rospy.sleep(2)

    tm.reset()
    tm.start()
    time_ex, time_now = 0, 0
    angle_ex = 0
    drive(0,0)
    boost = False
    one_time = True
    while True:
        while not image.size == (640*480*3):
            continue

        lpos, rpos = process_image(image)

        center = (lpos + rpos) / 2
        error = -((165+484)//2 + 0 - center)

        error = (error+20)*10.0/170

        tm.stop()
        time_now = tm.getTimeSec()
        tm.start()
        
        if error<0:
            error_val = - math.pow(-error,2.4)
        else:
            error_val = math.pow(error,2.4)

        angle = PID(error_val,0.5,0.0001,0.03)
        angle = max(min(50,angle),-50)
        if (abs(angle-angle_ex) > 70):
            if one_time:
                angle_ex = angle_ex * 0.2 + angle
                drive(angle_ex,15)
                one_time = False
                continue

        if (-5.8 < error < 5.8):
            if time_ex == 0:
                time_ex = time_now
            elif (time_now-time_ex>2):
                drive(angle,50)
            else:
                drive(angle,15)
        elif (-2.5 < error < 2.5):
            drive(angle, 15)
            time_ex=0
        else:
            drive(angle,11)
            time_ex=0

        angle_ex = angle

        one_time = True
    rospy.spin()

if __name__ == '__main__':

    start()
