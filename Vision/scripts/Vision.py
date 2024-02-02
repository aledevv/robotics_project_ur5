from pathlib import Path
import sys
import os
import rospy as ros
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import sensor_msgs.point_cloud2 as point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int32
from motion.msg import pos
from Block_detection import LegoBlock

# --------------------- DIRECTORIES and PATHS ---------------------
ROOT = Path(__file__).resolve().parents[1] # Vision
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))
ROOT = Path(os.path.abspath(ROOT))

ZED_IMG = str(ROOT) + '/images/img_ZED_cam.png'

R_cloud_to_world = np.matrix([[0, -0.499, 0.866], [-1, 0, 0], [0, -0.866, -0.499]])     # Transformation matrix Cloud Points -> World
x_c = np.array([-0.9, 0.24, -0.35])
base_offset = np.array([0.5, 0.35, 1.75])

TABLE_OFFSET = 0.86 + 0.1

# --------------------- Vision note class ---------------------
class Vision_node:

    def __init__(self):

        # Class parameters and flags
        self.block_list = []
        self.msg_list = []

        self.receive_next = True
        self.processing_img = False
        self.processing_point_cloud = False


        ros.init_node('vision', anonymous=True)

        # Publisher and Subscriber
        self.pos_publisher = ros.Publisher("/vision/pos", pos, queue_size=1)
        self.ack_pubscriber = ros.Publisher('/taskManager/stop', Int32, queue_size=1)

        self.img_subscriber = ros.Subscriber("/ur5/zed_node/left_raw/image_raw_color", Image, self.get_image)
        self.pointcloud_subscriber = ros.Subscriber("/ur5/zed_node/point_cloud/cloud_registered", PointCloud2,
                                                    self.get_point_cloud, queue_size=1)
        self.ack_subscriber = ros.Subscriber('/vision/ack', Int32, self.ackCallbak)

    def get_image(self, img_from_msg):
        if not self.processing_img:     # if vision node is not already busy
            self.receive_next = False   # block next images
            self.processing_img = True  # start to process images

            try:
                cv_obj = self.bridge.imgmsg_to_cv2(img_from_msg, "bgr8")
            except CvBridgeError as e:
                print(e)

        cv.imwrite(ZED_IMG, cv_obj)     # Saving zed img cam into images folder

        self.block_list = LegoBlock.detection(ZED_IMG)   # detection of blocks in the image

        self.processing_img = False

    def get_point_cloud(self, msg):
        if not self.processing_point_cloud: # if vision node is not already processing another point cloud

            self.processing_point_cloud = True

            if len(self.block_list) == 0:
                print("No blocks recognized")
                return

            for block in self.block_list:
                for data in point_cloud2.read_points(msg, field_names=['x','y','z'], skip_nans=True, uvs=block.block_center_pixels):
                    block.point_cloud_coordinates = (data[0], data[1], data[2])                 # storing coordinates of the block

                block.world_coordinates = R_cloud_to_world.dot(block.point_cloud_coordinates) + x_c + base_offset       # TODO refactor this

                block.info()        # print block's info

                self.send_msg(block)

            self.processing_point_cloud = False

    def send_msg(self, block):
        # Preparing a msg for Position Publisher (pos_publisher)

        msg = pos()
        msg.class_id = block.label

        # angles
        msg.pitch = 0
        msg.roll = 0
        msg.yaw = 0

        # world coordinates
        msg.x, msg.y, msg.z = block.world_coordinates[0]            # TODO not sure it works

        if msg.z < TABLE_OFFSET:
            self.msg_list.append(msg)

        if len(self.msg_list) > 0:
            current_msg = self.msg_list.pop()
            self.pos_pub.publish(current_msg)
            print('\nPublished:\n', current_msg)
        else:
            print('\nPUBLISHED ALL LEGO DETECTED\n')

        self.receive_next = True

    def ackCallbak(self, ack_ready):

        if self.receive_next == True and ack_ready.data == True:
            self.send_pos_msg()



if __name__ == '__main__':
    vn = Vision_node()

    try:
        ros.spin()
    except KeyboardInterrupt:
        print("VISION NODE KILLED")