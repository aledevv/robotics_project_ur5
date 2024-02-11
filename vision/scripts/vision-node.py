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
from robotics_project_ur5.srv import GetBrickPose, GetBrickPoseResponse
from geometry_msgs.msg import Pose

# --------------- DIRECTORIES ---------------
ROOT = Path(__file__).resolve().parents[1]  # vision directory
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
    # @Description Class to store points in the world space useful to estimate block pose
    # @Fields: coordinates x, y, z and pixels coordinates in ROI image

    def __init__(self):
        self.x = None
        self.y = None
        self.z = None
        self.px = ()

    def set(self, x, y, z, px):
        # @Description  Method to all parameters in once

        self.x = x
        self.y = y
        self.z = z
        self.px = px

    def is_min_x(self, coords, px):
        # @Description Compares this point with another to find the one with lower value of x
        # @Parameters tuple of world coordinates of second point and pixels coordinates of this latter

        if coords[0] < self.x:  # checks if x of 2nd point is lower than the one stored by this one,  if it does updates data
            self.x = coords[0]
            self.y = coords[1]
            self.z = coords[2]
            self.px = px

    def is_min_y(self, coords, px):
        # @Description Compares this point with another to find the one with lower value of y
        # @Parameters tuple of world coordinates of second point and pixels coordinates of this latter

        if coords[1] < self.y:  # checks if y of other point is lower than y of this object, if it does updates data
            self.x = coords[0]
            self.y = coords[1]
            self.z = coords[2]
            self.px = px

    def is_max_y(self, coords, px):
        # @Description Compares this point with another to find the one with greater value of y
        # @Parameters tuple of world coordinates of second point and pixels coordinates of this latter

        if coords[1] > self.y:  # checks whether y of other point is greater than this object, if it does updates data
            self.x = coords[0]
            self.y = coords[1]
            self.z = coords[2]
            self.px = px

    def info(self):
        # @Description Prints Point info
        print("x,y,z, PX: ", self.x, self.y, self.z, self.px)


def block_info(block):
    # @Description Prints Block info

    print("MESSAGE: ")
    print("\tlabel: " + str(block.label))
    print("\tx: " + str(block.x))
    print("\ty: " + str(block.y))
    print("\tz: " + str(block.z))
    print("\troll: " + str(block.roll))
    print("\tpitch: " + str(block.pitch))
    print("\tyaw: " + str(block.yaw))


def get_img(img):
    # @Description Callback function to store image from zed node and start detection process
    # @Parameters Image from Zed camera

    global can_acquire_img
    global can_take_point_cloud
    global block_list

    if can_acquire_img:
        try:
            cv_obj = bridge.imgmsg_to_cv2(img, "bgr8")      # convert image to cv2 obj
        except CvBridgeError as e:
            print(e)

        cv.imwrite(ZED_IMG, cv_obj)

        # FIND ROI
        roi.find_roi(ZED_IMG)           # find ROI

        # Detection phase
        block_list = Lego.detection(ROI_IMG)    # get list of detected blocks

        # Copying ROI image to draw line for pose estimations
        img = cv.imread(ROI_IMG, cv.IMREAD_COLOR)
        cv.imwrite(LINE_IMG, img)

        # FLAG TO CHANGE
        can_acquire_img = False
        can_take_point_cloud = True


def get_point_cloud(point_cloud):

    # @Description Callback function to collect point cloud and estimate center and pose of each block detected
    # @Parameters point cloud from zed node

    global can_take_point_cloud
    global block_list
    global measures

    if can_take_point_cloud:

        for block in block_list:
            # finding x, y, z of the center of the block leveraging center of the bounding boxes
            for data in point_cloud2.read_points(point_cloud, field_names=['x', 'y', 'z'], skip_nans=True, uvs=[block.center]):
                block.point_cloud_coord = [data[0], data[1], data[2]]

            # computing world coordinates through a transformation matrix and correcting result adding offsets
            block.world_coord = R_cloud_to_world.dot(block.point_cloud_coord) + x_camera + base_offset + block_offset
            block.world_coord[0, 2] = 0.86999       # z of the block is a constant

        # make 3 measures of block world coordinates
        if measures < 3:
            measures+=1
        else:
            can_take_point_cloud = False
            measures = 0

            # calculate the pose of the block
            for block in block_list:
                block.yaw = find_pose(point_cloud, block)

            # print block info
            Lego.print_block_info(block_list)

            # publish details to motion node
            msg_pub(block_list)


def find_pose(point_cloud, block):
    # @Description Function to compute object pose. Basically, the block is "sliced" at a specific height to find the 2
    #   useful points:
    #   - x_min: the nearest point the camera
    #   - y_min: the rightmost point (near the arm)
    #   these points are used to calculate yaw angle
    # @Parameters point cloud and the block
    # @Returns yaw angle (radiant)

    # CONSTANTS (used only here)
    table_height = 0.88
    target_height = 0.012 + table_height

    selected_points = []       # list of points at the target_height value

    print("Finding pose...")

    min_x = Point()
    min_y = Point()
    # max_y = Point()

    # scan point cloud of the bounding box
    for x in range(int(block.x1), int(block.x2)):
        for y in range(int(block.y1), int(block.y2)):
            for data in point_cloud2.read_points(point_cloud, field_names=['x', 'y', 'z'], skip_nans=True, uvs=[(x,y)]):
                point_coords = [data[0], data[1], data[2]]

                # transform current point to world coordinate
                point_world = R_cloud_to_world.dot(point_coords) + x_camera + base_offset

                # check whether it belongs to target height
                if abs(point_world[0, 2] - target_height) <= 0.001:

                    current_coords = (point_world[0, 0], point_world[0, 1], point_world[0, 2])
                    selected_points.append(current_coords)

                    if len(selected_points) == 1:   # if it is the first point store it as the min_x, min_y
                        color_pixel(x, y, 'red')
                        min_x.set(point_world[0, 0], point_world[0, 1], point_world[0, 2], (x, y))
                        min_y.set(point_world[0, 0], point_world[0, 1], point_world[0, 2], (x, y))
                        # max_y.set(point_world[0, 0], point_world[0, 1], point_world[0, 2], (x, y))
                    else:
                        color_pixel(x, y, 'red')
                        min_x.is_min_x(current_coords, (x, y))
                        min_y.is_min_y(current_coords, (x, y))
                        # max_y.is_max_y(current_coords, (x, y))

    # Print 3 vertices info
    #min_x.info()
    #min_y.info()
    #max_y.info()

    # Show in yellow points of interest
    color_pixel(min_x.px[0], min_x.px[1], 'yellow')
    color_pixel(min_y.px[0], min_y.px[1], 'yellow')
    # color_pixel(max_y.px[0], max_y.px[1], 'yellow')

    # Finding yaw angle
    yaw = math.atan2(min_y.x - min_x.x, min_x.y - min_y.y)

    return yaw


def color_pixel(x, y, color):
    # @Description function to color pixels in a image
    # @Parameters x and y coordinaets of pixels to color and a string indicating to the color to use

    img = cv.imread(LINE_IMG, cv.IMREAD_COLOR)

    if color == 'red':
        img[y][x] = np.array([0, 0, 255])
    elif color == 'yellow':
        img[y][x] = np.array([0, 255, 255])

    cv.imwrite(LINE_IMG, img)


def to_quaternions(r, p ,y):
    # @Description function transform RPY angles to Quaternions
    # @Parameters RPY angles
    # @Returns Quaternion

    return quaternion_from_euler(r, p, y)


def msg_pub(request):
    # @Description function that prepares and sends a message to motion node
    # @Parameters list of detected blocks

    #global send_next_msg
    global block_list
    #if send_next_msg:

    #msg = block()
    #q = to_quaternions(msg.roll, msg.pitch, msg.yaw)
       # if len(block_list) > 0:
        #    current_block = block_list.pop()
        #if len(block_list) == 0:
        #    print("PUBLISHED ALL BLOCKS")
        #    send_next_msg = False
        #    return
    poses = []
    labels = []
    pose = Pose()
        # Preparing msg
    for current_block in block_list:
        q = to_quaternions(0, 0, current_block.yaw)
        labels.append(current_block.label)
        pose.position.x = round(current_block.world_coord[0, 0], 6)
        pose.position.y = round(current_block.world_coord[0, 1], 6)
        pose.position.z = round(current_block.world_coord[0, 2], 6)
        
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        poses.append(pose)

    response = GetBrickPoseResponse()
    response.pose = poses
    response.label = labels
    response.numBricks = len(block_list)

    
    return response


        # msg.label = current_block.label
        # msg.x = round(current_block.world_coord[0, 0], 6)
        # msg.y = round(current_block.world_coord[0, 1], 6)
        # msg.z = round(current_block.world_coord[0, 2], 6)
        # msg.roll= 0.0
        # msg.pitch = 0.0
        # msg.yaw = current_block.yaw

        # QUATERNION conversion
    #q = to_quaternions(msg.roll, msg.pitch, msg.yaw)

        #block_info(msg)    # print msg info
    # poses = []
    # labels = []
    # response = GetBrickPoseResponse()
    # Pose().position.x = msg.x
    # Pose().position.y = msg.y
    # Pose().position.z = msg.z
    # Pose().orientation.z = q[3]
    # response.pose = Pose()
    # response.numBricks = len(block_list)
    # response.label = msg.label

  

        #pub.publish(msg)
        #rate.sleep()

        #send_next_msg = False
        #print("Waiting for sending next block")

# ----------------------------------------------- MAIN -----------------------------------------------


if __name__ == '__main__':

    # Publishers
    pub = rospy.Publisher('vision/position', block, queue_size=1)          # Vision msg publisher

    # Subscribers
    img_sub = rospy.Subscriber("/ur5/zed_node/left_raw/image_raw_color", Image, get_img)
    point_cloud_sub = rospy.Subscriber("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2, get_point_cloud, queue_size=1)   # Subscriber Point cloud

    #rospy.init_node('block_detector', anonymous=True)
    rospy.init_node('GetBrickPose')
    rate = rospy.Rate(10)  # 10hz
    #service = rospy.Service("GetBrickPose", GetBrickPose, msg_pub)
    service = rospy.Service("GetBrickPose", GetBrickPose, msg_pub)


    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


# import rospy
# from vision.msg import block
# from pathlib import Path
# import sys
# import os
# import numpy as np
# import cv2 as cv
# from cv_bridge import CvBridge, CvBridgeError
# from sensor_msgs.msg import Image
# import sensor_msgs.point_cloud2 as point_cloud2
# from sensor_msgs.msg import PointCloud2
# import Block_detection as Lego
# import Region_of_interest as roi
# from threading import Lock

# # --------------- DIRECTORIES ---------------
# ROOT = Path(__file__).resolve().parents[1] # Vision
# if str(ROOT) not in sys.path:
#     sys.path.append(str(ROOT))
# ROOT = Path(os.path.abspath(ROOT))

# ZED_IMG = str(ROOT) + '/images/img_ZED_cam.png'
# ROI_IMG = str(ROOT) + '/images/ROI_table.png'
# bridge = CvBridge()

# # --------------- WORLD PARAMS ---------------

# R_cloud_to_world = np.matrix([[0, -0.499, 0.866], [-1, 0, 0], [0, -0.866, -0.499]])
# x_c = np.array([-0.89, 0.24, -0.38])
# base_offset = np.array([0.5, 0.35, 1.75])


# TABLE_OFFSET = 0.86 + 0.1

# # --------------- FLAGS e GLOBALS ---------------
# can_acquire_img = True
# can_take_point_cloud = False
# send_next_msg = True
# block_list = []
# measures = 0        # num of measures for world position of blocks

# def block_info(block):
#     print("MESSAGE: ")
#     print("\tlabel: " + str(block.label))
#     print("\tx: " + str(block.x))
#     print("\ty: " + str(block.y))
#     print("\tz: " + str(block.z))
#     print("\troll: " + str(block.roll))
#     print("\tpitch: " + str(block.pitch))
#     print("\tyaw: " + str(block.yaw))


# def msg_pub(block_list):
#     global send_next_msg

#     if send_next_msg:

#         msg = block()
#         if len(block_list) > 0:
#             current_block = block_list.pop()
#         if len(block_list) == 0:
#             print("PUBLISHED ALL BLOCKS")
#             send_next_msg = False
#             return

#         msg.label = current_block.label
#         msg.x = round(current_block.world_coord[0, 0], 6)
#         msg.y = round(current_block.world_coord[0, 1], 6)
#         msg.z = round(current_block.world_coord[0, 2], 6)
#         msg.roll= 0.0
#         msg.pitch = 0.0
#         msg.yaw = 0.0
#         block_info(msg)

#         pub.publish(msg)
#         rate.sleep()

#         send_next_msg = False
#         print("Waiting for sending next block")




# def get_img(img):

#     global can_acquire_img
#     global can_take_point_cloud
#     global block_list

#     if can_acquire_img:
#         try:
#             cv_obj = bridge.imgmsg_to_cv2(img, "bgr8")
#         except CvBridgeError as e:
#             print(e)

#         cv.imwrite(ZED_IMG, cv_obj)

#         # FIND ROI
#         roi.find_roi(ZED_IMG)           # Only table is kept

#         # Detection phase
#         block_list = Lego.detection(ROI_IMG)



#         # FLAG TO CHANGE
#         can_acquire_img = False
#         can_take_point_cloud = True

#         print("Mo aspetto")

# def get_point_cloud(point_cloud):

#     global can_take_point_cloud
#     global block_list
#     global measures

#     if can_take_point_cloud:
#         point_list = []
#         for block in block_list:
#             for data in point_cloud2.read_points(point_cloud, field_names=['x', 'y', 'z'], skip_nans=True, uvs=[block.center]):
#                 point_list.append([data[0], data[1], data[2]])
#                 block.point_cloud_coord = [data[0], data[1], data[2]]

#             block.world_coord = R_cloud_to_world.dot(block.point_cloud_coord) + x_c + base_offset

#         # make 3 measures of block world coordinates
#         if measures < 3:
#             measures+=1
#         else:
#             can_take_point_cloud = False
#             measures = 0
#             Lego.print_block_info(block_list)
#             msg_pub(block_list)


# if __name__ == '__main__':

#     # Publishers
#     pub = rospy.Publisher('vision/position', block, queue_size=1)          # Vision msg publisher


#     # Subscribers
#     img_sub = rospy.Subscriber("/ur5/zed_node/left_raw/image_raw_color", Image, get_img)
#     point_cloud_sub = rospy.Subscriber("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2, get_point_cloud, queue_size=1)   # Subscriber Point cloud

#     rospy.init_node('block_detector', anonymous=True)
#     rate = rospy.Rate(10)  # 10hz


#     try:
#        rospy.spin()
#     except rospy.ROSInterruptException:
#        pass
