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

        self.ros_times = []
        self.ros_quats = []
        self.ros_pos   = []

        self.last_quat = np.array([0,0,0,0]) # x y z w 
        self.last_pos  = np.array([0,0,0])   # x y z 

        self.last_time    = 0.0 #time
        self.current_time = 0.0

    def linear_slerp(self, high_pos, low_pos, high_time, low_time, des_time): 

        num   = np.multiply(np.subtract(high_pos, low_pos),(des_time - low_time))
        denum = high_time - low_time

        return np.asarray(np.add(np.divide(num, denum), low_time))

    
    def calc_Omega(self, quat): #x y z w
        vector_norm = math.sqrt(quat[0]**2 + quat[1]**2 + quat[2]**2)
        return math.atan2(vector_norm, quat[3])

    def quat_pow(self, quat, Omega, pow):
        quat_v = [quat[0]/(1.0*math.sin(Omega)), 
                  quat[1]/(1.0*math.sin(Omega)),
                  quat[2]/(1.0*math.sin(Omega))] 

        powed_quat = np.array([quat_v[0]*math.sin(pow*Omega),
                               quat_v[1]*math.sin(pow*Omega),
                               quat_v[2]*math.sin(pow*Omega),
                               math.cos(pow*Omega)])

        return powed_quat


    def quaternion_slerp(self, high_quat, low_quat, high_time, low_time, des_time):

        quat_t1 = np.array(low_quat)
        quat_t2 = np.array(high_quat)

        print(quat_t1)
        t_hat = (des_time - low_time)/(high_time- low_time) # between 0 and 1

        quat_t1_omega = self.calc_Omega(quat_t1) #TODO: look into why it says it has 2 positional arguments
        quat_t1_m1    = self.quat_pow(quat_t1, quat_t1_omega, -1)

        qt1_inv_qt2  = self.quat_mul(quat_t1_m1, quat_t2)
        qt1_inv_qt2_omega = self.calc_Omega(qt1_inv_qt2)

        qt1_inv_qt2_t = self.quat_pow(qt1_inv_qt2, qt1_inv_qt2_omega, t_hat)
        

        est_quat = self.quat_mul(quat_t1,qt1_inv_qt2_t)
        return est_quat


    def quat_mul(self, quat_a, quat_b):
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

            self.ros_times.append(rosbag_time)
            self.ros_quats.append(np.array([msg.transform.rotation.x, msg.transform.rotation.y, 
                                           msg.transform.rotation.z, msg.transform.rotation.w]))
            self.ros_pos.append(np.array([msg.transform.translation.x, 
                                          msg.transform.translation.y, 
                                          msg.transform.rotation.z]))


    def build_interpolation_dataset(self, imu_times):
        high_index = 1
        low_index  = 0
        for time in imu_times: #expected to be more than the EC times
            
            if (time<self.ros_times[high_index] and time>=self.ros_times[low_index]): 
                low_quat = self.ros_quats[low_index]
                high_quat = self.ros_quats[high_index]

                low_pos = self.ros_pos[low_index]
                high_pos= self.ros_pos[high_index]

                low_time = self.ros_times[low_index]
                high_time = self.ros_times[high_index]

                point_interp = self.linear_slerp(high_pos, low_pos, high_time, low_time, time)
                quat_interp  = self.quaternion_slerp(high_quat, low_quat, high_time, low_time, time)

                self.VICON_MOVIE.append([point_interp, quat_interp,time])
                continue
            else:
                high_index += 1
                low_index  += 1

                low_quat = self.ros_quats[low_index]
                high_quat = self.ros_quats[high_index]

                low_pos = self.ros_pos[low_index]
                high_pos= self.ros_pos[high_index]

                low_time = self.ros_times[low_index]
                high_time = self.ros_times[high_index]

                point_interp = self.linear_slerp(high_pos, low_pos, high_time, low_time, time)
                quat_interp  = self.quaternion_slerp(high_quat, low_quat, high_time, low_time, time)

                self.VICON_MOVIE.append([point_interp, quat_interp,time])
                continue





if __name__ == '__main__':
    bag = rosbag.Bag('/home/joselavariega/bagfiles/test_7_jumps3.bag')
    counter = 0
    PREVIOUS_FRAME = 1
    my_Vicon = ViconPreprocessor()
    FRAMES_SECOND = 500

    my_Vicon.vicon_rosbag_data( bag, FRAMES_SECOND)