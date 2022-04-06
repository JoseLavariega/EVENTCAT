from numpy.lib.function_base import _quantile_unchecked
from EventCameraProcessing import EventCameraPreprocessor
import numpy as np
import matplotlib.pyplot as plt
#import scipy.signal as sps
import rosbag
import time
import math


class ViconPreprocessor:
    def __init__ (self):
        self.VICON_MOVIE = []
        self.VICON_ROSBAG_MOVIE = []
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

        self.quats_rpy = []

        self.est_velocities_moving_avg = []
        self.est_velocities_moving_avg_new = []
        self.est_velocities_local_differentiation = []
        self.est_velocities_kalman = []
        
        self.ang_rate_world = []
        self.ang_rate_body  = []
        self.unsmooth_ang_rate_world = []
        self.unsmooth_ang_rate_body  = []

        self.interp_times= []
        self.interp_quat = []
        self.interp_points= []

        self.all_v_R_b = []
        self.all_b_R_v = []

        self.noisy_quat_rates = []
        self.smoothed_quat_rates = []

        self.smoothed_quats =[]

        self.w_R_v = []

        self.rpy = []

    def linear_slerp(self, high_pos, low_pos, high_time, low_time, des_time): 

        num   = np.multiply(np.subtract(high_pos, low_pos),(des_time - low_time))
        denum = high_time - low_time

        return np.asarray(np.add(np.divide(num, denum), low_pos))

    
    def calc_Omega(self, quat): #x y z w
        vector_norm = math.sqrt(quat[0]**2 + quat[1]**2 + quat[2]**2)
        return math.atan2(vector_norm, quat[3]) #x,y,z,w

    def quat_pow(self, quat, Omega, pow):
        quat_v = [quat[0]/(1.0*math.sin(Omega)), 
                  quat[1]/(1.0*math.sin(Omega)),
                  quat[2]/(1.0*math.sin(Omega))] 

        powed_quat = np.array([quat_v[0]*math.sin(pow*Omega),
                               quat_v[1]*math.sin(pow*Omega),
                               quat_v[2]*math.sin(pow*Omega),
                               math.cos(pow*Omega)])

        return powed_quat #in x,y,z,w


    def quaternion_slerp(self, high_quat, low_quat, high_time, low_time, des_time):

        quat_t1 = np.array(low_quat)
        quat_t2 = np.array(high_quat)

        #print(quat_t1)
        t_hat = (des_time - low_time)/(high_time- low_time) # between 0 and 1

        quat_t1_omega = self.calc_Omega(quat_t1) 
        quat_t1_m1    = self.quat_pow(quat_t1, quat_t1_omega, -1)

        qt1_inv_qt2  = self.quat_mul(quat_t1_m1, quat_t2)
        qt1_inv_qt2_omega = self.calc_Omega(qt1_inv_qt2)

        qt1_inv_qt2_t = self.quat_pow(qt1_inv_qt2, qt1_inv_qt2_omega, t_hat)
        

        est_quat = self.quat_mul(quat_t1,qt1_inv_qt2_t)
        return est_quat #in x,y,z,w


    def quat_mul(self, quat_a, quat_b):
        xa, ya, za, wa = quat_a
        xb, yb, zb, wb = quat_b

        #returns  x,y,z,w
        return np.array([wa*xb + xa*wb + ya*zb - za*yb,
                         wa*yb - xa*zb + ya*wb + za*xb,
                         wa*zb + xa*yb - ya*xb + za*wb,
                         wa*wb - xa*xb - ya*yb - za*zb  ])


    def vicon_rosbag_data(self, bag, FRAMERATE):
        counter = 0
        PREVIOUS_FRAME = 1
        RATE_HERTZ = 1./FRAMERATE
        
        for topic, msg, t in bag.read_messages(topics=['/vicon/event_camera_body/event_camera_body']):
            if(PREVIOUS_FRAME >= 100):
               break

            if(counter == 0):
                ROSBAG_START = msg.header.stamp.secs + msg.header.stamp.nsecs*0.000000001
                counter += 1

                # get the first quaternion
                candidate_quat = np.array([msg.transform.rotation.x, msg.transform.rotation.y,
                                          msg.transform.rotation.z, msg.transform.rotation.w])

                self.w_R_v = self.quat_to_rotmat(candidate_quat).transpose()
                


            
            current_time = msg.header.stamp.secs + msg.header.stamp.nsecs*0.000000001
            rosbag_time = current_time - ROSBAG_START

            self.ros_times.append(rosbag_time)
            self.ros_quats.append(np.array([msg.transform.rotation.x, msg.transform.rotation.y, 
                                           msg.transform.rotation.z, msg.transform.rotation.w]))
            self.ros_pos.append(np.array([msg.transform.translation.x, 
                                          msg.transform.translation.y, 
                                          msg.transform.rotation.z]))

            PREVIOUS_FRAME += 1

        self.ros_pos = np.asarray(self.ros_pos)
        self.ros_quat = np.asarray(self.ros_quats)


    def build_interpolation_dataset(self, imu_times):
        high_index = 1
        low_index  = 0
        for time in imu_times: #expected to be more than the EC times
            
            if (time<self.ros_times[high_index] and time>=self.ros_times[low_index]): 

                #print('Entered here!')
                low_quat = self.ros_quats[low_index]
                high_quat = self.ros_quats[high_index]

                low_pos = self.ros_pos[low_index]
                high_pos= self.ros_pos[high_index]

                low_time = self.ros_times[low_index]
                high_time = self.ros_times[high_index]

                point_interp = self.linear_slerp(high_pos, low_pos, high_time, low_time, time)
                quat_interp  = self.quaternion_slerp(high_quat, low_quat, high_time, low_time, time) #x,y,z,w


                self.VICON_MOVIE.append([point_interp, quat_interp,time])
                self.interp_times.append(time)
                self.interp_points.append(point_interp)
                self.interp_quat.append(quat_interp)


                
            else:
                if (high_index + 1 < len(self.ros_quats)):
                    high_index += 1
                
                    low_index  += 1
                else:
                    high_index = high_index
                    low_index = low_index
                    print(high_index)
                    print(low_index)

                low_quat = self.ros_quats[low_index]
                high_quat = self.ros_quats[high_index]

                low_pos = self.ros_pos[low_index]
                high_pos= self.ros_pos[high_index]

                low_time = self.ros_times[low_index]
                high_time = self.ros_times[high_index]

                point_interp = self.linear_slerp(high_pos, low_pos, high_time, low_time, time)
                quat_interp  = self.quaternion_slerp(high_quat, low_quat, high_time, low_time, time)

                

                self.VICON_MOVIE.append([point_interp, quat_interp,time])

                self.interp_times.append(time)
                self.interp_points.append(point_interp)
                self.interp_quat.append(quat_interp)

                
                

        self.interp_points = np.asfarray(self.interp_points)

    def lin_moving_average(self,x,w):
        # w: range of moving average
        # x: our data
        x_data = x[:,0]
        y_data = x[:,1]
        z_data = x[:,2]

        x_conv = np.convolve(x_data, np.ones(w), 'valid')/w
        y_conv = np.convolve(y_data, np.ones(w), 'valid')/w
        z_conv = np.convolve(z_data, np.ones(w), 'valid')/w

        return np.asarray([x_conv, y_conv, z_conv]).T


    def quat_moving_average(self, quat, w):


        quat_i = quat[:,0]
        quat_j = quat[:,1]
        quat_k = quat[:,2]
        quat_scal = quat[:,3]

        i_conv = np.convolve(quat_i, np.ones(w), 'valid')/w
        j_conv = np.convolve(quat_j, np.ones(w), 'valid')/w
        k_conv = np.convolve(quat_k, np.ones(w), 'valid')/w
        scal_conv = np.convolve(quat_scal, np.ones(w), 'valid')/w

        return np.asarray([i_conv, j_conv, k_conv, scal_conv]).T


    def quat_to_rotmat(self, quat):
        # q0 - scalar
        # q1-3 vector

        #q_new = self.convert_quaternion_to_useful(quat) # may not be necessary
        #true quat indices are [scalar, i,j,k]
        q_new = np.asarray(quat).T #this line assumes quats come in as q0-2 vector, q3 sclar

        r_11 = q_new[0]**2 + q_new[1]**2 - q_new[2]**2 - q_new[3]**2
        r_12 = 2*q_new[1]*q_new[2] + 2*q_new[0]*q_new[3]
        r_13 = 2*q_new[1]*q_new[3] - 2*q_new[0]*q_new[2]

        r_21 = 2*q_new[1]*q_new[2] - 2*q_new[0]*q_new[3]
        r_22 = q_new[0]**2 - q_new[1]**2 + q_new[2]**2 -q_new[3]**2
        r_23 = 2*q_new[2]*q_new[3] + 2*q_new[0]*q_new[1]
        
        r_31 = 2*q_new[1]*q_new[3] + 2*q_new[0]*q_new[2]
        r_32 = 2*q_new[2]*q_new[3] - 2*q_new[0]*q_new[1]
        r_33 = q_new[0]**2 - q_new[1]**2 - q_new[2]**2 + q_new[3]**2

        rotation_matrix = np.array([[r_11, r_12, r_13],
                                    [r_21, r_22, r_23],
                                    [r_31, r_32, r_33]])

        return rotation_matrix



    def rate_matrix_body(self, quat):
        quat_scal = quat[3]
        quat_i    = quat[0]
        quat_j    = quat[1]
        quat_k    = quat[2]

        W_body = np.array([[-quat_i, quat_scal, quat_k, -quat_j],
                            [-quat_j, -quat_k, quat_scal, quat_i],
                            [-quat_k, quat_j, -quat_i, quat_scal]])

        return W_body

    def rate_matrix_world(self, quat):
        quat_scal = quat[3]
        quat_i    = quat[0]
        quat_j    = quat[1]
        quat_k    = quat[2]

        W_world = np.array([[-quat_i, quat_scal, quat_k, -quat_j],
                            [-quat_j, -quat_k, quat_scal, -quat_i],
                            [-quat_k, quat_j, -quat_i, quat_scal]])

        return W_world

    def convert_quaternion_to_useful(self, quat):
        quat_scal = quat[3]
        quat_i    = quat[0]
        quat_j    = quat[1]
        quat_k    = quat[2]

        return([quat_scal, quat_i, quat_j, quat_k])


    def velocity_estimator(self):
        #noisy_positions = self.interp_points.copy()
        times           = self.interp_times.copy()
        noisy_positions = self.interp_points.copy()


        #noisy_positions = self.lin_moving_average(self.interp_points.copy(),5)
        # calculate all differencess locally
        for i in range(1,len(noisy_positions)):
            local_velocity = (noisy_positions[i] - noisy_positions[i-1])/(times[i]-times[i-1])


            
            self.est_velocities_local_differentiation.append(local_velocity)

        #cast as array again
        self.est_velocities_local_differentiation = np.asarray(self.est_velocities_local_differentiation)

        # implement a moving average
        self.est_velocities_moving_avg = self.lin_moving_average(self.est_velocities_local_differentiation, 10)

        for i in range(len(self.est_velocities_moving_avg)):
            local_vel = self.est_velocities_moving_avg[i]
            transformed_vel = np.matmul(self.all_b_R_v[i], local_vel)

            self.est_velocities_moving_avg_new.append(transformed_vel)
            


    def kf_velestimator(self):
        time_step = 0.002
        A_mat     = np.array([[1,0,0,time_step,0,0],
                              [0,1,0,0,time_step,0],
                              [0,0,1,0,0,time_step],
                              [0,0,0,1,0,0],
                              [0,0,0,0,1,0],
                              [0,0,0,0,0,1]])

        C_mat     = np.array([[1,0,0,0,0,0],
                              [0,1,0,0,0,0],
                              [0,0,1,0,0,0]])

        



    def ang_rate_estimator(self):
        noisy_quats = np.array(self.interp_quat.copy() )
        quat_times  = self.interp_times.copy()
        noisy_quat_rates = []

        # build up q_dot
        for i in range(1, len(noisy_quats)):
            noisy_quat_rates.append((noisy_quats[i] - noisy_quats[i-1])/(quat_times[i]- quat_times[i-1]))

        self.noisy_quat_rates = np.array(noisy_quat_rates)
        #maybe smooth over noisy quat rates?
        self.smoothed_quat_rates = np.array(self.quat_moving_average(self.noisy_quat_rates.copy(),10))
        self.smoothed_quats = np.array(self.quat_moving_average(self.interp_quat.copy(),10))

        for i in range(0, len(self.smoothed_quat_rates)):
            body_matrix = self.rate_matrix_body(noisy_quats[i+1])
            world_matrix = self.rate_matrix_world(noisy_quats[i+1])
            #quaternion = self.convert_quaternion_to_useful(noisy_quats[i+1])


            quaternion = self.smoothed_quats[i+1]
            print(quaternion)
            v_R_b = self.quat_to_rotmat(quaternion)
            b_R_v = v_R_b.transpose

            self.all_v_R_b.append(v_R_b)
            self.all_b_R_v.append(b_R_v)
            

            #local_velocity_trans = self.all_b_R_v[i-1]*local_velocity

            #print(np.shape(body_matrix))
            #print(np.shape(self.smoothed_quat_rates[i].T))

            #print(body_matrix)

            #smoothed_quat_rates[i].reshape((4,1))
            

            ang_rate_world = 2*np.matmul(world_matrix, self.smoothed_quat_rates[i])
            ang_rate_body_local  = 2*np.matmul(body_matrix, self.smoothed_quat_rates[i])

            ang_rate_world_noise = 2*np.matmul(world_matrix, self.noisy_quat_rates[i])
            ang_rate_body_noise  = 2*np.matmul(body_matrix, self.noisy_quat_rates[i])


            #multiply angular rates by transform matrix
            ang_rate_body_final = np.matmul(b_R_v,ang_rate_body_local)
            

            self.ang_rate_world.append(ang_rate_world)
            self.ang_rate_body.append(ang_rate_body_final)

            self.unsmooth_ang_rate_world.append(ang_rate_world_noise)
            self.unsmooth_ang_rate_body.append(ang_rate_body_noise)

        self.ang_rate_world = np.array(self.ang_rate_world)
        self.ang_rate_body  = np.array(self.ang_rate_body)
        self.unsmooth_ang_rate_world = np.array(self.unsmooth_ang_rate_world)
        self.unsmooth_ang_rate_body  = np.array(self.unsmooth_ang_rate_body)

    def rot_mat_to_rpy(self, rotmat):
        r11 = rotmat[0][0]
        r12 = rotmat[0][1]
        r13 = rotmat[0][2]
        r21 = rotmat[1][0]
        r22 = rotmat[1][1]
        r23 = rotmat[1][2]
        r31 = rotmat[2][0]
        r32 = rotmat[2][1]
        r33 = rotmat[2][2]

        roll = math.atan2(r23,r33)
        pitch= -math.asin(r13)
        yaw  = math.atan2(r12,r11)

        return np.array([roll, pitch, yaw])

    def build_vicon_data_movie(self):

        for i in range(len(self.all_v_R_b)):
            rotmat = np.matmul(self.w_R_v,self.arr_v_R_b[i])
            self.rpy.append(self.rotm_mat_to_rpy(rotmat))
            

        self.VICON_ROSBAG_MOVIE = [np.asarray(self.interp_times), 
                                    np.asarray(self.rpy), #DONE!
                                    np.asarray(self.ang_rate_body), #DONE!
                                    np.asarray(self.est_velocities_moving_avg_new)] #DONE!
    



if __name__ == '__main__':
    bag = rosbag.Bag('/home/joselavariega/bagfiles/test_7_jumps3.bag')
    counter = 0
    PREVIOUS_FRAME = 1
    my_Vicon = ViconPreprocessor()
    FRAMES_SECOND = 500

    my_Vicon.vicon_rosbag_data( bag, FRAMES_SECOND)