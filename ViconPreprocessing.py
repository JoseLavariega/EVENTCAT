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

        self.last_quat = np.array([0,0,0,0]) # x y z w 
        self.last_pos  = np.array([0,0,0])   # x y z 

        self.last_time    = 0.0 #time
        self.current_time = 0.0

    def linear_slerp(self, current_pos, des_time): 

        num   = np.multiply(np.subtract(current_pos, self.last_pos),(des_time - self.last_time))
        denum = self.current_time - self.last_time

        return np.asarray(np.add(np.divide(num, denum), self.last_pos))
    
    def calc_Omega(quat): #x y z w
        vector_norm = math.sqrt(quat[0]**2 + quat[1]**2 + quat[2]**2)
        return math.atan2(vector_norm, quat[3])

    def quat_pow(quat, Omega, pow):
        quat_v = [quat[0]/(1.0*math.sin(Omega)), 
                  quat[1]/(1.0*math.sin(Omega)),
                  quat[2]/(1.0*math.sin(Omega))] 

        powed_quat = np.array([quat_v[0]*math.sin(pow*Omega),
                               quat_v[1]*math.sin(pow*Omega),
                               quat_v[2]*math.sin(pow*Omega),
                               math.cos(pow*Omega)])

        return powed_quat


    def quaternion_slerp(self, current_quat, des_time):

        quat_t1 = self.last_quat
        quat_t2 = current_quat

        t_hat = (des_time - self.last_time)/(self.current_time - self.last_time) # between 0 and 1

        quat_t1_omega = self.calc_Omega(quat_t1)
        quat_t1_m1    = self.quat_pow(quat_t1, quat_t1_omega, -1)

        qt1_inv_qt2  = self.quat_mul(quat_t1_m1, quat_t2)
        qt1_inv_qt2_omega = self.calc_Omega(qt1_inv_qt2)

        qt1_inv_qt2_t = self.quat_pow(qt1_inv_qt2, qt1_inv_qt2_omega, t_hat)
        

        est_quat = self.quat_mul(quat_t1,qt1_inv_qt2_t)
        return est_quat


    def quat_mul(quat_a, quat_b):
        xa, ya, za, wa = quat_a
        xb, yb, zb, wb = quat_b

        return np.array([wa*xb + xa*wb + ya*zb - za*yb,
                         wa*yb - xa*zb + ya*wb + za*xb,
                         wa*zb + xa*yb - ya*xb + za*wb,
                         wa*wb - xa*xb - ya*yb - za*zb  ])


    def vicon_rosbag_data(self, bag, FRAMERATE):
        counter = 0
        PREVIOUS_FRAME = 1
        RATE_HERTZ = 1./FRAMERATE
        
        for topic, msg, t in bag.read_messages(topics=['/vicon/event_camera_body/event_camera_body']):
            if(PREVIOUS_FRAME >= 750):
                break

            if(counter == 0):
                ROSBAG_START = msg.header.stamp.secs + msg.header.stamp.nsecs*0.000000001
                counter += 1
            
            current_time = msg.header.stamp.secs + msg.header.stamp.nsecs*0.000000001
            rosbag_time = current_time - ROSBAG_START

            if(rosbag_time >= PREVIOUS_FRAME * RATE_HERTZ):
                p_x = msg.transform.translation.x
                p_y = msg.transform.translation.y
                p_z = msg.transform.translation.z

                q_x = msg.transform.rotation.x
                q_y = msg.transform.rotation.y
                q_z = msg.transform.rotation.z
                q_w = msg.transform.rotation.w

                #pos_x.append(p_x)
                #pos_y.append(p_y)
                #pos_z.append(p_z)
                #quat_x.append(q_x)
                #quat_x.append(q_y)
                #quat_x.append(q_z)
                #quat_w.append(q_w)





if __name__ == '__main__':
    bag = rosbag.Bag('/home/joselavariega/bagfiles/test_7_jumps3.bag')
    counter = 0
    PREVIOUS_FRAME = 1
    my_Vicon = ViconPreprocessor()
    FRAMES_SECOND = 500