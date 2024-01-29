import cv2
import numpy as np
from pathlib import Path
import sys
import os

# --------------------- DIRECTORIES and PATHS ---------------------
ROOT = Path(__file__).resolve().parents[1] # Vision folder
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))
ROOT = Path(os.path.abspath(ROOT))

INPUT_FILE = str(ROOT) + '/log/img_ZED_cam.png'
OUTPUT_DIR = str(ROOT) + '/log/ROI_table.png'

# --------------------- FUNCTIONS  ---------------------


def find_roi(img):
    mask = np.zeros(img.shape[0:2], dtype=np.uint8) # Mask has the same dimensions of zed cam image
    points = np.array([[[844, 410], [1196, 410], [1541, 905], [678, 905]]]) # corners of the table
    cv2.drawContours(mask, [points], -1, (255, 255, 255), -1, cv2.LINE_AA)
    res = cv2.bitwise_and(img, img, mask=mask)
    cv2.imwrite(OUTPUT_DIR, res)



# --------------------- MAIN  ---------------------
if __name__ == '__main__':

    image = cv2.imread(INPUT_FILE)

    find_roi(image)
