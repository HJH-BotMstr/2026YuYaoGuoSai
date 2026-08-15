from xgolib import XGO
from xgoedu import XGOEDU
import time
import cv2
import numpy as np
import threading
import time
import math

dog = XGO(port='/dev/ttyAMA0', version="xgomini")
XGO_edu = XGOEDU()
dog.reset()

