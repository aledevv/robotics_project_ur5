import cv2
import numpy as np
from pathlib import Path
import sys
import os

# --------------------- DIRECTORIES and PATHS ---------------------
ROOT = Path(__file__).resolve().parents[1]  # vision folder
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))
ROOT = Path(os.path.abspath(ROOT))

INPUT_FILE = str(ROOT) + '/images/img_ZED_cam.png'
OUTPUT_FILE = str(ROOT) + '/images/ROI_table.png'

# --------------------- FUNCTIONS  ---------------------


def find_roi(img_path):
    ''' @brief Finds the ROI knowing the position of the vertices of the table through a bitwise operation between input image and a mask
        @param path of input image
    '''

    img = cv2.imread(img_path)
    mask = np.zeros(img.shape[0:2], dtype=np.uint8)
    table_points = np.array([[[844, 410], [1196, 410], [1541, 905], [678, 905]]]) # corners of the table
    cv2.drawContours(mask, [table_points], -1, (255, 255, 255), -1, cv2.LINE_AA)
    res = cv2.bitwise_and(img, img, mask=mask)
    cv2.imwrite(OUTPUT_FILE, res)


