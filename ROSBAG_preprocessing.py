import numpy as np
import matplotlib.pyplot as plt
import rosbag
import time
import math

from EventCameraProcessing import EventCameraPreprocessor
from ViconPreprocessing import ViconPreprocessor
from IMUProcessing import IMUPreprocessor


if __name__ == '__main__':
    bag = rosbag.Bag('/home/joselavariega/bagfiles/test_7_jumps3.bag')
    my_ec    = EventCameraPreprocessor()
    my_imu   = IMUPreprocessor()
    my_vicon = ViconPreprocessor()
    