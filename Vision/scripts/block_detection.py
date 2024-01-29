from pathlib import Path
import sys
import os
import torch
from matplotlib import pyplot as plt
import numpy as np
import cv2 as cv
from IPython.display import display
from PIL import Image
import region_of_interest as roi
from ultralytics import YOLO
from ultralytics.utils.torch_utils import select_device
import json

# --------------------- DIRECTORIES and PATHS ---------------------
ROOT = Path(__file__).resolve().parents[1] # Vision folder
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))
ROOT = Path(os.path.abspath(ROOT))

IMG_PATH = str(ROOT) + '/log/ROI_table.png'

# --------------------- CONSTANTS ---------------------
WEIGHTS = str(ROOT) + "/weights/best.pt"
CONFIDENCE = 0.5
yolo_model = YOLO(WEIGHTS) # YOLO model trained to detect lego pieces

SAVE_PREDICTION_TXT = False
SAVE_PREDICTION_IMG = False

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
        self.label = label
        self.confidence = confidence
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2


def store_blocks(data):

    blocks = []

    for lego in data:
        current_block = LegoBlock(lego["name"], lego["confidence"],
                                  lego["box"]["x1"], lego["box"]["y1"],
                                  lego["box"]["x2"], lego["box"]["y2"])
        blocks.append(current_block)

    return blocks


def print_blocks_info(blocks):

    for block in blocks:
        print("Block label: " + block.label)
        print("\tconfidence: " + str(round(block.confidence, 3)))
        print("\tTop-left corner: (" + str(int(block.x1)) + ", " + str(int(block.y1)) + ")")
        print("\tBottom-right corner: (" + str(int(block.x2)) + ", " + str(int(block.y2)) + ")")



def detection(img_path):

    roi.find_roi(img_path)

    print("Detecting...")
    results = yolo_model.predict(source=roi.OUTPUT_FILE, line_width=1, conf=0.5, save_txt=SAVE_PREDICTION_TXT, save=SAVE_PREDICTION_IMG)

    lego_blocs_json = []
    for result in results:
        data = json.loads(result.tojson())

    blocks = store_blocks(data)

    return blocks


# CLI:      python detection.py /path/to/img.smth
if __name__ == '__main__':

    if len(sys.argv) > 1:   # img has been passe via CLI
        img = sys.argv[1]
    else:
        img = roi.INPUT_FILE

    block_list = detection(img)

    print_blocks_info(block_list)