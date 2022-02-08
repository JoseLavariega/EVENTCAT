from EventCameraProcessing import EventCameraPreprocessor
import numpy as np
import matplotlib.pyplot as plt
import rosbag
import time
import math

class ViconPreprocessor:
    def __init__ (self):
        self.VICON_MOVIE = []
        self.START_TIME  = 0.0

        #initialization
        self.pos_x = []
        self.pos_y = []
        self.pos_z = []

        self.quat_x = []
        self.quat_y = []
        self.quat_z = []
        self.quat_w = []

    def linear_slerp(body):
        print('linear slerp!')

    def quaternion_slerp():
        print('quaternion slerp!')

    def vicon_rosbag_data(self, bag, FRAMERATE):
        counter = 0
        PREVIOUS_FRAME = 1
        RATE_HERTZ = 1./FRAMERATE
        
        for topic, msg, t in bag.read_messages(topics=['/vicon/event_camera_body/event_camera_body']):
            print('topic!')



if __name__ == '__main__':
    bag = rosbag.Bag('/home/joselavariega/bagfiles/test_7_jumps3.bag')
    counter = 0
    PREVIOUS_FRAME = 1
    my_Vicon = ViconPreprocessor()
    FRAMES_SECOND = 500