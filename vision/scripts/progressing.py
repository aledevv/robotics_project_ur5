import rospy
from vision.msg import block
from pathlib import Path
import sys
import os
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import sensor_msgs.point_cloud2 as point_cloud2
from sensor_msgs.msg import PointCloud2
import Block_detection as Lego
import Region_of_interest as roi
from threading import Lock

# --------------- DIRECTORIES ---------------
ROOT = Path(__file__).resolve().parents[1] # Vision
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))
ROOT = Path(os.path.abspath(ROOT))

ZED_IMG = str(ROOT) + '/images/img_ZED_cam.png'
ROI_IMG = str(ROOT) + '/images/ROI_table.png'
bridge = CvBridge()

# --------------- WORLD PARAMS ---------------

R_cloud_to_world = np.matrix([[0, -0.499, 0.866], [-1, 0, 0], [0, -0.866, -0.499]])
x_c = np.array([-0.89, 0.24, -0.38])
base_offset = np.array([0.5, 0.35, 1.75])


TABLE_OFFSET = 0.86 + 0.1

# --------------- FLAGS e GLOBALS ---------------
can_acquire_img = True
can_take_point_cloud = False
send_next_msg = True
block_list = []
measures = 0        # num of measures for world position of blocks

def block_info(block):
    print("MESSAGE: ")
    print("\tlabel: " + str(block.label))
    print("\tx: " + str(block.x))
    print("\ty: " + str(block.y))
    print("\tz: " + str(block.z))
    print("\troll: " + str(block.roll))
    print("\tpitch: " + str(block.pitch))
    print("\tyaw: " + str(block.yaw))


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
        msg.yaw = 0.0
        block_info(msg)

        pub.publish(msg)
        rate.sleep()

        send_next_msg = False
        print("Waiting for sending next block")




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



        # FLAG TO CHANGE
        can_acquire_img = False
        can_take_point_cloud = True

        print("Mo aspetto")

def get_point_cloud(point_cloud):

    global can_take_point_cloud
    global block_list
    global measures

    if can_take_point_cloud:
        point_list = []
        for block in block_list:
            for data in point_cloud2.read_points(point_cloud, field_names=['x', 'y', 'z'], skip_nans=True, uvs=[block.center]):
                point_list.append([data[0], data[1], data[2]])
                block.point_cloud_coord = [data[0], data[1], data[2]]

            block.world_coord = R_cloud_to_world.dot(block.point_cloud_coord) + x_c + base_offset

        # make 3 measures of block world coordinates
        if measures < 3:
            measures+=1
        else:
            can_take_point_cloud = False
            measures = 0
            Lego.print_block_info(block_list)
            msg_pub(block_list)


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
