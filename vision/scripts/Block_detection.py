from pathlib import Path
import sys
import os
import torch
from matplotlib import pyplot as plt
import numpy as np
import cv2 as cv
from IPython.display import display
from PIL import Image
import Region_of_interest as roi
from ultralytics import YOLO
from ultralytics.utils.torch_utils import select_device
import json



# --------------------- DIRECTORIES and PATHS ---------------------
ROOT = Path(__file__).resolve().parents[1]  # vision folder
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))
ROOT = Path(os.path.abspath(ROOT))

IMG_PATH = str(ROOT) + '/images/ROI_table.png'

# --------------------- CONSTANTS ---------------------
WEIGHTS = str(ROOT) + "/weights/best.pt"
CONFIDENCE = 0.5
yolo_model = YOLO(WEIGHTS) # YOLO model trained to detect lego pieces

SAVE_PREDICTION_TXT = False
SAVE_PREDICTION_IMG = True

LEGO_LABELS =  ['X1-Y1-Z2',
                'X1-Y2-Z1',
                'X1-Y2-Z2',
                'X1-Y2-Z2-CHAMFER',
                'X1-Y2-Z2-TWINFILLET',
                'X1-Y3-Z2',
                'X1-Y3-Z2-FILLET',
                'X1-Y4-Z1',
                'X1-Y4-Z2',
                'X2-Y2-Z2',
                'X2-Y2-Z2-FILLET']


class LegoBlock:
    
    def __init__(self, label, confidence, x1, y1, x2, y2):
        '''
        @ param label block class
        @ param confidence detection confidence
        @ param x1, y1 left top corner of bbox
        @ param x2, y2 right bottom corner of bbox
        '''
        self.label = label
        self.confidence = confidence
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.yaw = None
        self.img = Image.open(IMG_PATH)
        self.center = (int((x1 + x2) / 2), int((y1 + y2) / 2))
        self.point_cloud_coord = ()
        self.world_coord = ()

    def info(self):
        '''
        @brief Function that writes some info about the block
        
        '''
        
        print("\nBlock label: " + self.label)
        print("\tconfidence: " + str(round(self.confidence, 3)))
        print("\tTop-left corner: (" + str(int(self.x1)) + ", " + str(int(self.y1)) + ")")
        print("\tBottom-right corner: (" + str(int(self.x2)) + ", " + str(int(self.y2)) + ")")
        print("\tWorld coordinates: " + str(self.world_coord))
        print("\t Yaw angle: " + str(self.yaw))


def store_blocks(data):

    ''' 
      @brief takes blocks data from json object from YOLO detection to create a list of blocks
      @param JSON/dictionary object
      @retval a list of Blocks
    '''
    blocks = []

    for lego in data:
        current_block = LegoBlock(lego["name"], lego["confidence"],
                                  lego["box"]["x1"], lego["box"]["y1"],
                                  lego["box"]["x2"], lego["box"]["y2"])
        blocks.append(current_block)

    return blocks


def print_block_info(blocks):

    ''' @brief prints list of blocks info
        @param List of Block objects
    '''

    for block in blocks:
        block.info()


def detection(img_path):
    '''
    @brief It crops the ZED-cam image into the Region of Interest (only the table), block detection with YOLO follows
    @param path of the input image (from ZED camera)
    @retval the list of detected Blocks
    '''

    roi.find_roi(img_path)

    print("Detecting...")
    results = yolo_model.predict(source=roi.OUTPUT_FILE, line_width=1, conf=0.5, save_txt=SAVE_PREDICTION_TXT, save=SAVE_PREDICTION_IMG)

    for result in results:
        data = json.loads(result.tojson())

    blocks = store_blocks(data)

    return blocks



# CLI:  python detection.py /path/to/img.smth
if __name__ == '__main__':

    if len(sys.argv) > 1:   # img has been passed via CLI
        img = sys.argv[1]
    else:
        img = roi.INPUT_FILE

    block_list = detection(img)

    print_blocks_info(block_list)
