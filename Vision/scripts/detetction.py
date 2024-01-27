from pathlib import Path
import sys
import os
#import torch
from matplotlib import pyplot as plt
import numpy as np
import cv2 as cv
from IPython.display import display
from PIL import Image
#from RegionOfInterest import RegionOfInterest
from ultralytics import YOLO

# --------------------- DIRECTORIES and PATHS ---------------------
ROOT = Path(__file__).resolve().parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))
VISION_DIR = os.path.abspath(os.path.join(ROOT, ".."))
ROI_IMG = os.path.abspath(os.path.join(ROOT, "log/img_ROI.png"))

# --------------------- CONSTANTS ---------------------
WEIGHTS = os.path.join(VISION_DIR, "weights/best.pt")
confidence = 0.5
yolo_model = YOLO(WEIGHTS) # YOLO model trained to detect lego pieces

LEGO_LABELS = [ 'X1-Y1-Z2',
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

# --------------------- CLASSES  ---------------------

# DETECTOR
# TODO finish the detector
# LEGO PIECE
# TODO finish lego piece object containing its inferred info
if __name__ == '__main__':

    # TEST to see if it works
   # img = 'img_ROI.png'
   # results = yolo_model.predict(source=img, line_width=1, conf=0.25, save_txt=True, save=True)

    pass
