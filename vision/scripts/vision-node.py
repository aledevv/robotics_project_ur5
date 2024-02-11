import rospy
from vision.msg import block
from pathlib import Path
import sys
import os
import copy
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import sensor_msgs.point_cloud2 as point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
import Block_detection as Lego
import Region_of_interest as roi
import math
from tf.transformations import quaternion_from_euler

# --------------- DIRECTORIES ---------------
ROOT = Path(__file__).resolve().parents[1] # Vision
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))
ROOT = Path(os.path.abspath(ROOT))

ZED_IMG = str(ROOT) + '/images/img_ZED_cam.png'
ROI_IMG = str(ROOT) + '/images/ROI_table.png'
LINE_IMG = str(ROOT) + '/images/line-height.png'
bridge = CvBridge()

# --------------- WORLD PARAMS ---------------

R_cloud_to_world = np.matrix([[0, -0.49948, 0.86632], [-1., 0., 0.], [0., -0.86632, -0.49948]])
x_camera = np.array([-0.9, 0.24, -0.35])
base_offset = np.array([0.5, 0.35, 1.75])
block_offset = [0.0189, -0.007, 0]


TABLE_OFFSET = 0.86 + 0.1

# --------------- FLAGS e GLOBALS ---------------
can_acquire_img = True
can_take_point_cloud = False
send_next_msg = True
block_list = []
measures = 0        # num of measures for world position of blocks

class Point:
    def __init__(self):
        self.x = None
        self.y = None
        self.z = None
        self.px = ()

    def set(self, x, y, z, px):
        self.x = x
        self.y = y
        self.z = z
        self.px = px

    def is_min_x(self, coords, px):
        if coords[0] < self.x:
            self.x = coords[0]
            self.y = coords[1]
            self.z = coords[2]
            self.px = px

    def is_min_y(self, coords, px):
        if coords[1] < self.y:
            self.x = coords[0]
            self.y = coords[1]
            self.z = coords[2]
            self.px = px

    def is_max_y(self, coords, px):
        if coords[1] > self.y:
            self.x = coords[0]
            self.y = coords[1]
            self.z = coords[2]
            self.px = px

    def info(self):
        print("x,y,z, PX: ", self.x, self.y, self.z, self.px)

def block_info(block):
    print("MESSAGE: ")
    print("\tlabel: " + str(block.label))
    print("\tx: " + str(block.x))
    print("\ty: " + str(block.y))
    print("\tz: " + str(block.z))
    print("\troll: " + str(block.roll))
    print("\tpitch: " + str(block.pitch))
    print("\tyaw: " + str(block.yaw))


def get_img(img):

    global can_acquire_img
    global can_take_point_cloud
    global block_list

    if can_acquire_img:
        try:
            cv_obj = bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv.imwrite(ZED_IMG, cv_obj)

        # FIND ROI
        roi.find_roi(ZED_IMG)           # Only table is kept

        # Detection phase
        block_list = Lego.detection(ROI_IMG)

        # Copying ROI image to draw line for pose estimations
        img = cv.imread(ROI_IMG, cv.IMREAD_COLOR)
        cv.imwrite(str(ROOT) + "/images/line-height.png", img)

        # FLAG TO CHANGE
        can_acquire_img = False
        can_take_point_cloud = True

def get_point_cloud(point_cloud):

    global can_take_point_cloud
    global block_list
    global measures

    if can_take_point_cloud:

        for block in block_list:
            for data in point_cloud2.read_points(point_cloud, field_names=['x', 'y', 'z'], skip_nans=True, uvs=[block.center]):
                block.point_cloud_coord = [data[0], data[1], data[2]]

            block.world_coord = R_cloud_to_world.dot(block.point_cloud_coord) + x_camera + base_offset + block_offset
            block.world_coord[0, 2] = 0.86999



        # make 3 measures of block world coordinates
        if measures < 3:
            measures+=1
        else:
            can_take_point_cloud = False
            measures = 0

            for block in block_list:
                block.yaw = find_pose(point_cloud, block)

            Lego.print_block_info(block_list)

            msg_pub(block_list)


def find_pose(point_cloud, block):
    table_height = 0.88
    target_height = 0.012 + table_height

    selected_points = []

    print("Finding pose...")

    min_x = Point()
    min_y = Point()
    max_y = Point()

    # scan pointcloud of the bounding box
    for x in range(int(block.x1), int(block.x2)):
        for y in range(int(block.y1), int(block.y2)):
            for data in point_cloud2.read_points(point_cloud, field_names=['x', 'y', 'z'], skip_nans=True, uvs=[(x,y)]):
                point_coords = [data[0], data[1], data[2]]
                point_world = R_cloud_to_world.dot(point_coords) + x_camera + base_offset
                if abs(point_world[0, 2] - target_height) <= 0.001:

                    current_coords = (point_world[0, 0], point_world[0, 1], point_world[0, 2])
                    selected_points.append(current_coords)

                    if len(selected_points) == 1:
                        color_pixel(x, y, 'red')
                        min_x.set(point_world[0, 0], point_world[0, 1], point_world[0, 2], (x, y))
                        min_y.set(point_world[0, 0], point_world[0, 1], point_world[0, 2], (x, y))
                        max_y.set(point_world[0, 0], point_world[0, 1], point_world[0, 2], (x, y))
                    else:
                        color_pixel(x, y, 'red')
                        min_x.is_min_x(current_coords, (x, y))
                        min_y.is_min_y(current_coords, (x, y))
                        max_y.is_max_y(current_coords, (x, y))

    # Print 3 vertices info
    #min_x.info()
    #min_y.info()
    #max_y.info()

    color_pixel(min_x.px[0], min_x.px[1], 'yellow')
    color_pixel(min_y.px[0], min_y.px[1], 'yellow')
    color_pixel(max_y.px[0], max_y.px[1], 'yellow')

    # Finding yaw angle
    yaw = math.atan2(min_y.x - min_x.x, min_x.y - min_y.y)

    return yaw



def color_pixel(x, y, color):

    img = cv.imread(LINE_IMG, cv.IMREAD_COLOR)

    if color == 'red':
        img[y][x] = np.array([0, 0, 255])
    elif color == 'yellow':
        img[y][x] = np.array([0, 255, 255])
    #cv.imshow("Image", img_copy)
    #cv.waitKey(0)
    cv.imwrite(str(ROOT)+"/images/line-height.png", img);

def to_quaternions(r, p ,y):
    return quaternion_from_euler(r, p, y)

# Function to send msg
def msg_pub(block_list):
    global send_next_msg

    if send_next_msg:

        msg = block()
        if len(block_list) > 0:
            current_block = block_list.pop()
        if len(block_list) == 0:
            print("PUBLISHED ALL BLOCKS")
            send_next_msg = False
            return

        msg.label = current_block.label
        msg.x = round(current_block.world_coord[0, 0], 6)
        msg.y = round(current_block.world_coord[0, 1], 6)
        msg.z = round(current_block.world_coord[0, 2], 6)
        msg.roll= 0.0
        msg.pitch = 0.0
        msg.yaw = current_block.yaw

        # QUATERNION
        q = to_quaternions(msg.roll, msg.pitch, msg.yaw)

        #block_info(msg)

        pub.publish(msg)
        rate.sleep()

        send_next_msg = False
        print("Waiting for sending next block")

# ----------------------------------------------- MAIN -----------------------------------------------

if __name__ == '__main__':

    # Publishers
    pub = rospy.Publisher('vision/position', block, queue_size=1)          # Vision msg publisher


    # Subscribers
    img_sub = rospy.Subscriber("/ur5/zed_node/left_raw/image_raw_color", Image, get_img)
    point_cloud_sub = rospy.Subscriber("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2, get_point_cloud, queue_size=1)   # Subscriber Point cloud

    rospy.init_node('block_detector', anonymous=True)
    rate = rospy.Rate(10)  # 10hz


    try:
       rospy.spin()
    except rospy.ROSInterruptException:
       pass
