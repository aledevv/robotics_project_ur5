import cv2
import numpy as np
from pathlib import Path
import sys
import os

# --------------------- DIRECTORIES and PATHS ---------------------
ROOT = Path(__file__).resolve().parents[0]
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))

INPUT_FILE = os.path.join(ROOT, '/log/img_ZED_cam.png')
OUTPUT_DIR = os.path.join(ROOT, '/log/')

# --------------------- REGION OF INTEREST CLASS ---------------------


class ROI:

    def __int__(self):
        self.img = cv2.imread(INPUT_FILE)

    def find_roi(self):
        mask = np.zeros(self.img.shape[0:2], dtype=np.uint8)
        points = np.array([[[845, 409], [1201, 412], [1545, 913], [658, 921]]])
        cv2.drawContours(mask, [points], -1, (255, 255, 255), -1, cv2.LINE_AA)
        res = cv2.bitwise_and(self.img, self.img, mask=mask)
        cv2.imwrite(OUTPUT_DIR, res)
        cv2.destroyAllWindows()


if __name__ == '__main__':
    roi = ROI()
    roi.find_roi()
