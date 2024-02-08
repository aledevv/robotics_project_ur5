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
import Block_detection as lego
import Region_of_interest as roi

# --------------- DIRECTORIES ---------------
ROOT = Path(__file__).resolve().parents[1] # Vision
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))
ROOT = Path(os.path.abspath(ROOT))

ZED_IMG = str(ROOT) + '/images/img_ZED_cam.png'
ROI_IMG = str(ROOT) + '/images/ROI_table.png'

def block_info(block):
    print("MESSAGE: ")
    print("\tlabel: " + str(block.label))
    print("\tx: " + str(block.x))
    print("\ty: " + str(block.y))
    print("\tz: " + str(block.z))
    print("\troll: " + str(block.roll))
    print("\tpitch: " + str(block.pitch))
    print("\tyaw: " + str(block.yaw))


def msg_pub():
    pub = rospy.Publisher('position', block, queue_size=1)
    rospy.init_node('vision', anonymous=True)
    rate = rospy.Rate(10) # 10hz


    msg = block()
    while not rospy.is_shutdown():
        msg.label = 0
        msg.x = 1.0
        msg.y = 2.0
        msg.z = 3.0
        msg.roll= 0.1
        msg.pitch = 0.2
        msg.yaw = 0.3
        block_info(msg)
        
        pub.publish(msg)
        rate.sleep()

def get_img():
    # TAKE IMG FROM SUBSCRIBER and store it in ZED_IMG Path
    roi.find_roi(ZED_IMG)                                           # Only table is kept
    pass # TODO STORE IMG FROM ZED CAM in usual PATH

def get_point_cloud():
    pass # TODO PATH in a file in Windows Desktop

def vision():

    get_img()
    get_point_cloud()

    blocks = lego.detection(ROI_IMG)

    msg_pub()


if __name__ == '__main__':
   try:
       msg_pub()
   except rospy.ROSInterruptException:
       pass
