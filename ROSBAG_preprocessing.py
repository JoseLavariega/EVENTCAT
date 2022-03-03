import numpy as np
import matplotlib.pyplot as plt
import rosbag
import time
import math
import pickle as pk

from EventCameraProcessing import EventCameraPreprocessor
from ViconPreprocessing import ViconPreprocessor
from IMUProcessing import IMUPreprocessor


if __name__ == '__main__':
        # all of our data!
        bag1 = rosbag.Bag('/home/joselavariega/bagfiles/test_1_facing_back.bag') #15 seconds           
        
        FRAMERATE = 500

        #all_bagfiles = [bag1, bag2, bag3, bag4, bag5, bag6, bag7, bag8, bag9, bag10, bag11, bag12, bag13, bag14, bag15, bag16, bag17, bag18]

        my_ec1    = EventCameraPreprocessor()
        my_imu1   = IMUPreprocessor()
        my_vicon1 = ViconPreprocessor()

        
        my_imu1.rosbag_populate_network_imu(bag1,FRAMERATE)
        my_ec1.rosbag_populate_network_frames(bag1,FRAMERATE)
        my_vicon1.vicon_rosbag_data(bag1,FRAMERATE)

        bag1.close()
        my_vicon1.build_interpolation_dataset(my_imu1.times)

        my_vicon1.velocity_estimator()
        my_vicon1.ang_rate_estimator()
        my_vicon1.build_vicon_data_movie()

        my_movie_FULL1 = [my_ec1.CORNER_EVENTS_MOVIE, my_imu1.IMU_MOVIE, my_vicon1.VICON_ROSBAG_MOVIE]

        pk.dump(my_ec1.CORNER_EVENTS_MOVIE, open('rosbag1_ec','wb'))
        pk.dump(my_imu1.IMU_MOVIE, open('rosbag1_imu','wb')) 
        pk.dump(my_vicon1.VICON_ROSBAG_MOVIE, open('rosbag1_vicon','wb')) 
        pk.dump(my_movie_FULL1, open('rosbag1_full','wb')) 

        bag2 = rosbag.Bag('/home/joselavariega/bagfiles/test_1_facing_down.bag') #11 seconds

        my_ec2    = EventCameraPreprocessor()
        my_imu2   = IMUPreprocessor()
        my_vicon2 = ViconPreprocessor()

        
        my_imu2.rosbag_populate_network_imu(bag2,FRAMERATE)
        my_ec2.rosbag_populate_network_frames(bag2,FRAMERATE)
        my_vicon2.vicon_rosbag_data(bag2,FRAMERATE)

        bag2.close()
        my_vicon2.build_interpolation_dataset(my_imu2.times)

        my_vicon2.velocity_estimator()
        my_vicon2.ang_rate_estimator()
        my_vicon2.build_vicon_data_movie()

        my_movie_FULL2 = [my_ec2.CORNER_EVENTS_MOVIE, my_imu2.IMU_MOVIE, my_vicon2.VICON_ROSBAG_MOVIE]

        pk.dump(my_ec2.CORNER_EVENTS_MOVIE, open('rosbag2_ec','wb'))
        pk.dump(my_imu2.IMU_MOVIE, open('rosbag2_imu','wb')) 
        pk.dump(my_vicon2.VICON_ROSBAG_MOVIE, open('rosbag2_vicon','wb')) 
        pk.dump(my_movie_FULL2, open('rosbag2_full','wb'))

        

        bag3 = rosbag.Bag('/home/joselavariega/bagfiles/test_1_facing_front.bag') #6 seconds

        my_ec3    = EventCameraPreprocessor()
        my_imu3   = IMUPreprocessor()
        my_vicon3 = ViconPreprocessor()

        
        my_imu3.rosbag_populate_network_imu(bag3,FRAMERATE)
        my_ec3.rosbag_populate_network_frames(bag3,FRAMERATE)
        my_vicon3.vicon_rosbag_data(bag3,FRAMERATE)

        bag3.close()
        my_vicon3.build_interpolation_dataset(my_imu3.times)

        my_vicon3.velocity_estimator()
        my_vicon3.ang_rate_estimator()
        my_vicon3.build_vicon_data_movie()

        my_movie_FULL3 = [my_ec3.CORNER_EVENTS_MOVIE, my_imu3.IMU_MOVIE, my_vicon3.VICON_ROSBAG_MOVIE]

        pk.dump(my_ec3.CORNER_EVENTS_MOVIE, open('rosbag3_ec','wb'))
        pk.dump(my_imu3.IMU_MOVIE, open('rosbag3_imu','wb')) 
        pk.dump(my_vicon3.VICON_ROSBAG_MOVIE, open('rosbag3_vicon','wb')) 
        pk.dump(my_movie_FULL3, open('rosbag3_full','wb'))




        bag4 = rosbag.Bag('/home/joselavariega/bagfiles/test_1_facing_right.bag') #13 seconds, shows movement on EC but no IMU
        
        my_ec4    = EventCameraPreprocessor()
        my_imu4   = IMUPreprocessor()
        my_vicon4 = ViconPreprocessor()

        
        my_imu4.rosbag_populate_network_imu(bag4,FRAMERATE)
        my_ec4.rosbag_populate_network_frames(bag4,FRAMERATE)
        my_vicon4.vicon_rosbag_data(bag4,FRAMERATE)

        bag4.close()
        my_vicon4.build_interpolation_dataset(my_imu4.times)

        my_vicon4.velocity_estimator()
        my_vicon4.ang_rate_estimator()
        my_vicon4.build_vicon_data_movie()

        my_movie_FULL4 = [my_ec4.CORNER_EVENTS_MOVIE, my_imu4.IMU_MOVIE, my_vicon4.VICON_ROSBAG_MOVIE]

        pk.dump(my_ec4.CORNER_EVENTS_MOVIE, open('rosbag4_ec','wb'))
        pk.dump(my_imu4.IMU_MOVIE, open('rosbag4_imu','wb')) 
        pk.dump(my_vicon4.VICON_ROSBAG_MOVIE, open('rosbag4_vicon','wb')) 
        pk.dump(my_movie_FULL4, open('rosbag4_full','wb'))
        
        
        
        bag5 = rosbag.Bag('/home/joselavariega/bagfiles/test_3_updown_facedown_2.bag') # 36 seconds

        my_ec5    = EventCameraPreprocessor()
        my_imu5   = IMUPreprocessor()
        my_vicon5 = ViconPreprocessor()

        
        my_imu5.rosbag_populate_network_imu(bag5,FRAMERATE)
        my_ec5.rosbag_populate_network_frames(bag5,FRAMERATE)
        my_vicon5.vicon_rosbag_data(bag5,FRAMERATE)

        bag5.close()
        my_vicon5.build_interpolation_dataset(my_imu5.times)

        my_vicon5.velocity_estimator()
        my_vicon5.ang_rate_estimator()
        my_vicon5.build_vicon_data_movie()

        my_movie_FULL5 = [my_ec5.CORNER_EVENTS_MOVIE, my_imu5.IMU_MOVIE, my_vicon5.VICON_ROSBAG_MOVIE]

        pk.dump(my_ec5.CORNER_EVENTS_MOVIE, open('rosbag5_ec','wb'))
        pk.dump(my_imu5.IMU_MOVIE, open('rosbag5_imu','wb')) 
        pk.dump(my_vicon5.VICON_ROSBAG_MOVIE, open('rosbag5_vicon','wb')) 
        pk.dump(my_movie_FULL5, open('rosbag5_full','wb'))


        bag6 = rosbag.Bag('/home/joselavariega/bagfiles/test_3_updown_facedown.bag') # 50 seconds

        my_ec6    = EventCameraPreprocessor()
        my_imu6   = IMUPreprocessor()
        my_vicon6 = ViconPreprocessor()

        
        my_imu6.rosbag_populate_network_imu(bag6,FRAMERATE)
        my_ec6.rosbag_populate_network_frames(bag6,FRAMERATE)
        my_vicon6.vicon_rosbag_data(bag6,FRAMERATE)

        bag6.close()
        my_vicon6.build_interpolation_dataset(my_imu1.times)

        my_vicon6.velocity_estimator()
        my_vicon6.ang_rate_estimator()
        my_vicon6.build_vicon_data_movie()

        my_movie_FULL6 = [my_ec6.CORNER_EVENTS_MOVIE, my_imu6.IMU_MOVIE, my_vicon6.VICON_ROSBAG_MOVIE]

        pk.dump(my_ec6.CORNER_EVENTS_MOVIE, open('rosbag6_ec','wb'))
        pk.dump(my_imu6.IMU_MOVIE, open('rosbag6_imu','wb')) 
        pk.dump(my_vicon6.VICON_ROSBAG_MOVIE, open('rosbag6_vicon','wb')) 
        pk.dump(my_movie_FULL6, open('rosbag6_full','wb'))




        bag7 = rosbag.Bag('/home/joselavariega/bagfiles/test_4_random_walk.bag') #58 seconds
        my_ec7    = EventCameraPreprocessor()
        my_imu7   = IMUPreprocessor()
        my_vicon7 = ViconPreprocessor()

        
        my_imu7.rosbag_populate_network_imu(bag7,FRAMERATE)
        my_ec7.rosbag_populate_network_frames(bag7,FRAMERATE)
        my_vicon7.vicon_rosbag_data(bag7,FRAMERATE)

        bag7.close()
        my_vicon7.build_interpolation_dataset(my_imu7.times)

        my_vicon7.velocity_estimator()
        my_vicon7.ang_rate_estimator()
        my_vicon7.build_vicon_data_movie()

        my_movie_FULL7 = [my_ec7.CORNER_EVENTS_MOVIE, my_imu7.IMU_MOVIE, my_vicon7.VICON_ROSBAG_MOVIE]

        pk.dump(my_ec7.CORNER_EVENTS_MOVIE, open('rosbag7_ec','wb'))
        pk.dump(my_imu7.IMU_MOVIE, open('rosbag7_imu','wb')) 
        pk.dump(my_vicon7.VICON_ROSBAG_MOVIE, open('rosbag7_vicon','wb')) 
        pk.dump(my_movie_FULL7, open('rosbag7_full','wb'))




        bag8 = rosbag.Bag('/home/joselavariega/bagfiles/test_5_updown.bag') # 23 seconds
        my_ec8   = EventCameraPreprocessor()
        my_imu8   = IMUPreprocessor()
        my_vicon8 = ViconPreprocessor()

        
        my_imu8.rosbag_populate_network_imu(bag8,FRAMERATE)
        my_ec8.rosbag_populate_network_frames(bag8,FRAMERATE)
        my_vicon8.vicon_rosbag_data(bag8,FRAMERATE)

        bag8.close()
        my_vicon8.build_interpolation_dataset(my_imu8.times)

        my_vicon8.velocity_estimator()
        my_vicon8.ang_rate_estimator()
        my_vicon8.build_vicon_data_movie()

        my_movie_FULL8= [my_ec8.CORNER_EVENTS_MOVIE, my_imu8.IMU_MOVIE, my_vicon8.VICON_ROSBAG_MOVIE]

        pk.dump(my_ec8.CORNER_EVENTS_MOVIE, open('rosbag8_ec','wb'))
        pk.dump(my_imu8.IMU_MOVIE, open('rosbag8_imu','wb')) 
        pk.dump(my_vicon8.VICON_ROSBAG_MOVIE, open('rosbag8_vicon','wb')) 
        pk.dump(my_movie_FULL8, open('rosbag8_full','wb'))




        bag9 = rosbag.Bag('/home/joselavariega/bagfiles/test_6_circular.bag') # 27 seconds\
        my_ec9    = EventCameraPreprocessor()
        my_imu9   = IMUPreprocessor()
        my_vicon9 = ViconPreprocessor()

        
        my_imu9.rosbag_populate_network_imu(bag9,FRAMERATE)
        my_ec9.rosbag_populate_network_frames(bag9,FRAMERATE)
        my_vicon9.vicon_rosbag_data(bag9,FRAMERATE)

        bag9.close()
        my_vicon9.build_interpolation_dataset(my_imu9.times)

        my_vicon9.velocity_estimator()
        my_vicon9.ang_rate_estimator()
        my_vicon9.build_vicon_data_movie()

        my_movie_FULL9 = [my_ec9.CORNER_EVENTS_MOVIE, my_imu9.IMU_MOVIE, my_vicon9.VICON_ROSBAG_MOVIE]

        pk.dump(my_ec9.CORNER_EVENTS_MOVIE, open('rosbag9_ec','wb'))
        pk.dump(my_imu9.IMU_MOVIE, open('rosbag9_imu','wb')) 
        pk.dump(my_vicon9.VICON_ROSBAG_MOVIE, open('rosbag9_vicon','wb')) 
        pk.dump(my_movie_FULL9, open('rosbag9_full','wb'))


        bag10 = rosbag.Bag('/home/joselavariega/bagfiles/test_6_circular3.bag')# 43 seconds
        my_ec10    = EventCameraPreprocessor()
        my_imu10   = IMUPreprocessor()
        my_vicon10 = ViconPreprocessor()

        
        my_imu10.rosbag_populate_network_imu(bag10,FRAMERATE)
        my_ec10.rosbag_populate_network_frames(bag10,FRAMERATE)
        my_vicon10.vicon_rosbag_data(bag10,FRAMERATE)

        bag10.close()
        my_vicon10.build_interpolation_dataset(my_imu10.times)

        my_vicon10.velocity_estimator()
        my_vicon10.ang_rate_estimator()
        my_vicon10.build_vicon_data_movie()

        my_movie_FULL10 = [my_ec10.CORNER_EVENTS_MOVIE, my_imu10.IMU_MOVIE, my_vicon10.VICON_ROSBAG_MOVIE]

        pk.dump(my_ec10.CORNER_EVENTS_MOVIE, open('rosbag10_ec','wb'))
        pk.dump(my_imu10.IMU_MOVIE, open('rosbag10_imu','wb')) 
        pk.dump(my_vicon10.VICON_ROSBAG_MOVIE, open('rosbag10_vicon','wb')) 
        pk.dump(my_movie_FULL10, open('rosbag10_full','wb'))

        bag11 = rosbag.Bag('/home/joselavariega/bagfiles/test_7_jumps.bag')#38 seconds
        my_ec11    = EventCameraPreprocessor()
        my_imu11   = IMUPreprocessor()
        my_vicon11 = ViconPreprocessor()

        
        my_imu11.rosbag_populate_network_imu(bag11,FRAMERATE)
        my_ec11.rosbag_populate_network_frames(bag11,FRAMERATE)
        my_vicon11.vicon_rosbag_data(bag11,FRAMERATE)

        bag11.close()
        my_vicon11.build_interpolation_dataset(my_imu11.times)

        my_vicon11.velocity_estimator()
        my_vicon11.ang_rate_estimator()
        my_vicon11.build_vicon_data_movie()

        my_movie_FULL11 = [my_ec11.CORNER_EVENTS_MOVIE, my_imu11.IMU_MOVIE, my_vicon11.VICON_ROSBAG_MOVIE]

        pk.dump(my_ec11.CORNER_EVENTS_MOVIE, open('rosbag11_ec','wb'))
        pk.dump(my_imu11.IMU_MOVIE, open('rosbag11_imu','wb')) 
        pk.dump(my_vicon11.VICON_ROSBAG_MOVIE, open('rosbag11_vicon','wb')) 
        pk.dump(my_movie_FULL11, open('rosbag11_full','wb'))


        bag12 = rosbag.Bag('/home/joselavariega/bagfiles/test_7_jumps3.bag')# 30 seconds
        my_ec12   = EventCameraPreprocessor()
        my_imu12   = IMUPreprocessor()
        my_vicon12 = ViconPreprocessor()

        
        my_imu12.rosbag_populate_network_imu(bag12,FRAMERATE)
        my_ec12.rosbag_populate_network_frames(bag12,FRAMERATE)
        my_vicon12.vicon_rosbag_data(bag12,FRAMERATE)

        bag12.close()
        my_vicon12.build_interpolation_dataset(my_imu12.times)

        my_vicon12.velocity_estimator()
        my_vicon12.ang_rate_estimator()
        my_vicon12.build_vicon_data_movie()

        my_movie_FULL12 = [my_ec12.CORNER_EVENTS_MOVIE, my_imu12.IMU_MOVIE, my_vicon12.VICON_ROSBAG_MOVIE]

        pk.dump(my_ec12.CORNER_EVENTS_MOVIE, open('rosbag12_ec','wb'))
        pk.dump(my_imu12.IMU_MOVIE, open('rosbag12_imu','wb')) 
        pk.dump(my_vicon12.VICON_ROSBAG_MOVIE, open('rosbag12_vicon','wb')) 
        pk.dump(my_movie_FULL12, open('rosbag12_full','wb'))



        bag13 = rosbag.Bag('/home/joselavariega/bagfiles/test_8_jog.bag') # 37 seconds
        my_ec13    = EventCameraPreprocessor()
        my_imu13  = IMUPreprocessor()
        my_vicon13 = ViconPreprocessor()

        
        my_imu13.rosbag_populate_network_imu(bag13,FRAMERATE)
        my_ec13.rosbag_populate_network_frames(bag13,FRAMERATE)
        my_vicon13.vicon_rosbag_data(bag13,FRAMERATE)

        bag13.close()
        my_vicon13.build_interpolation_dataset(my_imu13.times)

        my_vicon13.velocity_estimator()
        my_vicon13.ang_rate_estimator()
        my_vicon13.build_vicon_data_movie()

        my_movie_FULL13 = [my_ec13.CORNER_EVENTS_MOVIE, my_imu13.IMU_MOVIE, my_vicon13.VICON_ROSBAG_MOVIE]

        pk.dump(my_ec13.CORNER_EVENTS_MOVIE, open('rosbag13_ec','wb'))
        pk.dump(my_imu13.IMU_MOVIE, open('rosbag13_imu','wb')) 
        pk.dump(my_vicon13.VICON_ROSBAG_MOVIE, open('rosbag13_vicon','wb')) 
        pk.dump(my_movie_FULL13, open('rosbag13_full','wb'))



        bag14 = rosbag.Bag('/home/joselavariega/bagfiles/test_9_taps.bag') #27 seconds
        my_ec14    = EventCameraPreprocessor()
        my_imu14   = IMUPreprocessor()
        my_vicon14 = ViconPreprocessor()

        
        my_imu14.rosbag_populate_network_imu(bag14,FRAMERATE)
        my_ec14.rosbag_populate_network_frames(bag14,FRAMERATE)
        my_vicon14.vicon_rosbag_data(bag14,FRAMERATE)

        bag14.close()
        my_vicon14.build_interpolation_dataset(my_imu14.times)

        my_vicon14.velocity_estimator()
        my_vicon14.ang_rate_estimator()
        my_vicon14.build_vicon_data_movie()

        my_movie_FULL14 = [my_ec14.CORNER_EVENTS_MOVIE, my_imu14.IMU_MOVIE, my_vicon14.VICON_ROSBAG_MOVIE]

        pk.dump(my_ec14.CORNER_EVENTS_MOVIE, open('rosbag14_ec','wb'))
        pk.dump(my_imu14.IMU_MOVIE, open('rosbag14_imu','wb')) 
        pk.dump(my_vicon14.VICON_ROSBAG_MOVIE, open('rosbag14_vicon','wb')) 
        pk.dump(my_movie_FULL14, open('rosbag14_full','wb'))



        bag15 = rosbag.Bag('/home/joselavariega/bagfiles/test_9_taps2.bag') # 8 seconds
        my_ec15    = EventCameraPreprocessor()
        my_imu15   = IMUPreprocessor()
        my_vicon15 = ViconPreprocessor()

        
        my_imu15.rosbag_populate_network_imu(bag15,FRAMERATE)
        my_ec15.rosbag_populate_network_frames(bag15,FRAMERATE)
        my_vicon15.vicon_rosbag_data(bag15,FRAMERATE)

        bag15.close()
        my_vicon15.build_interpolation_dataset(my_imu15.times)

        my_vicon15.velocity_estimator()
        my_vicon15.ang_rate_estimator()
        my_vicon15.build_vicon_data_movie()

        my_movie_FULL15 = [my_ec15.CORNER_EVENTS_MOVIE, my_imu15.IMU_MOVIE, my_vicon15.VICON_ROSBAG_MOVIE]

        pk.dump(my_ec15.CORNER_EVENTS_MOVIE, open('rosbag15_ec','wb'))
        pk.dump(my_imu15.IMU_MOVIE, open('rosbag15_imu','wb')) 
        pk.dump(my_vicon15.VICON_ROSBAG_MOVIE, open('rosbag15_vicon','wb')) 
        pk.dump(my_movie_FULL15, open('rosbag15_full','wb'))




        bag16 = rosbag.Bag('/home/joselavariega/bagfiles/test_9_taps3.bag') # 55 seconds
        my_ec16    = EventCameraPreprocessor()
        my_imu16  = IMUPreprocessor()
        my_vicon16 = ViconPreprocessor()

        
        my_imu16.rosbag_populate_network_imu(bag16,FRAMERATE)
        my_ec16.rosbag_populate_network_frames(bag16,FRAMERATE)
        my_vicon16.vicon_rosbag_data(bag16,FRAMERATE)

        bag16.close()
        my_vicon16.build_interpolation_dataset(my_imu16.times)

        my_vicon16.velocity_estimator()
        my_vicon16.ang_rate_estimator()
        my_vicon16.build_vicon_data_movie()

        my_movie_FULL16 = [my_ec16.CORNER_EVENTS_MOVIE, my_imu16.IMU_MOVIE, my_vicon16.VICON_ROSBAG_MOVIE]

        pk.dump(my_ec16.CORNER_EVENTS_MOVIE, open('rosbag16_ec','wb'))
        pk.dump(my_imu16.IMU_MOVIE, open('rosbag16_imu','wb')) 
        pk.dump(my_vicon16.VICON_ROSBAG_MOVIE, open('rosbag16_vicon','wb')) 
        pk.dump(my_movie_FULL16, open('rosbag16_full','wb'))




        bag17 = rosbag.Bag('/home/joselavariega/bagfiles/test_9_taps4.bag') # 22 seconds
        my_ec17    = EventCameraPreprocessor()
        my_imu17   = IMUPreprocessor()
        my_vicon17 = ViconPreprocessor()

        
        my_imu17.rosbag_populate_network_imu(bag17,FRAMERATE)
        my_ec17.rosbag_populate_network_frames(bag17,FRAMERATE)
        my_vicon17.vicon_rosbag_data(bag17,FRAMERATE)

        bag17.close()
        my_vicon17.build_interpolation_dataset(my_imu17.times)

        my_vicon17.velocity_estimator()
        my_vicon17.ang_rate_estimator()
        my_vicon17.build_vicon_data_movie()

        my_movie_FULL17 = [my_ec17.CORNER_EVENTS_MOVIE, my_imu17.IMU_MOVIE, my_vicon17.VICON_ROSBAG_MOVIE]

        pk.dump(my_ec17.CORNER_EVENTS_MOVIE, open('rosbag17_ec','wb'))
        pk.dump(my_imu17.IMU_MOVIE, open('rosbag17_imu','wb')) 
        pk.dump(my_vicon17.VICON_ROSBAG_MOVIE, open('rosbag17_vicon','wb')) 
        pk.dump(my_movie_FULL17, open('rosbag17_full','wb'))





        bag18 = rosbag.Bag('/home/joselavariega/bagfiles/test_9_taps5.bag') # 41 seconds
        my_ec18    = EventCameraPreprocessor()
        my_imu18   = IMUPreprocessor()
        my_vicon18 = ViconPreprocessor()

        
        my_imu18.rosbag_populate_network_imu(bag18,FRAMERATE)
        my_ec18.rosbag_populate_network_frames(bag18,FRAMERATE)
        my_vicon18.vicon_rosbag_data(bag18,FRAMERATE)

        bag18.close()
        my_vicon18.build_interpolation_dataset(my_imu18.times)

        my_vicon18.velocity_estimator()
        my_vicon18.ang_rate_estimator()
        my_vicon18.build_vicon_data_movie()

        my_movie_FULL18 = [my_ec18.CORNER_EVENTS_MOVIE, my_imu18.IMU_MOVIE, my_vicon18.VICON_ROSBAG_MOVIE]

        pk.dump(my_ec18.CORNER_EVENTS_MOVIE, open('rosbag18_ec','wb'))
        pk.dump(my_imu18.IMU_MOVIE, open('rosbag18_imu','wb')) 
        pk.dump(my_vicon18.VICON_ROSBAG_MOVIE, open('rosbag18_vicon','wb')) 
        pk.dump(my_movie_FULL18, open('rosbag18_full','wb'))





        print('Done!')


        












        '''
        #Actually build up the thing
        print(len(my_ec1.CORNER_EVENTS_MOVIE))
        print(len(my_imu1.IMU_MOVIE))
        print(len(my_vicon1.VICON_MOVIE))

        print(len(my_vicon1.interp_times))
        print(len(my_vicon1.ang_rate_body))
        print(len(my_vicon1.est_velocities_moving_avg))
        print(len(my_vicon1.interp_quat))
        '''

        #print('points')
        #print(my_vicon.interp_points)
        #print('quats')
        #print(my_vicon.interp_quat)
        #print('times')
        #print(my_vicon.interp_times)

        #print(my_imu.times)

        #print(len(my_vicon1.VICON_ROSBAG_MOVIE))


        # plotting
        #print(my_vicon.est_velocities_local_differentiation)
        #print(str(my_vicon.est_velocities_local_differentiation.size))

        #print(my_vicon.ros_times)

        '''
        plt.figure(1)
        plt.plot(my_vicon.interp_times[:len(my_vicon.est_velocities_local_differentiation)], my_vicon.est_velocities_local_differentiation[:,0], 'm--',
        my_vicon.interp_times[:len(my_vicon.est_velocities_moving_avg)], my_vicon.est_velocities_moving_avg[:,0], 'r',
        my_vicon.interp_times[:len(my_vicon.est_velocities_local_differentiation)], my_vicon.est_velocities_local_differentiation[:,1], 'y--',
        my_vicon.interp_times[:len(my_vicon.est_velocities_moving_avg)], my_vicon.est_velocities_moving_avg[:,1], 'g',
        my_vicon.interp_times[:len(my_vicon.est_velocities_local_differentiation)], my_vicon.est_velocities_local_differentiation[:,2], 'c--',
        my_vicon.interp_times[:len(my_vicon.est_velocities_moving_avg)], my_vicon.est_velocities_moving_avg[:,2], 'b',
        )

        #my_vicon.interp_times, my_vicon.interp_points[:,0], 'g'

        plt.figure(2)
        plt.plot(my_vicon.interp_times[:len(my_vicon.ang_rate_body)], my_vicon.ang_rate_body[:,0], 'r',
        my_vicon.interp_times[:len(my_vicon.unsmooth_ang_rate_body)], my_vicon.unsmooth_ang_rate_body[:,0], 'm--',
        my_vicon.interp_times[:len(my_vicon.ang_rate_body)], my_vicon.ang_rate_body[:,1], 'g',
        my_vicon.interp_times[:len(my_vicon.unsmooth_ang_rate_body)], my_vicon.unsmooth_ang_rate_body[:,1], 'y--',
        my_vicon.interp_times[:len(my_vicon.ang_rate_body)], my_vicon.ang_rate_body[:,2], 'b',
        my_vicon.interp_times[:len(my_vicon.unsmooth_ang_rate_body)], my_vicon.unsmooth_ang_rate_body[:,2], 'c--')

        plt.figure(3)
        plt.plot(my_vicon.interp_times[:len(my_vicon.noisy_quat_rates)], my_vicon.noisy_quat_rates[:,0], 'r',
        my_vicon.interp_times[:len(my_vicon.smoothed_quat_rates)], my_vicon.smoothed_quat_rates[:,0], '--m',
        my_vicon.interp_times[:len(my_vicon.noisy_quat_rates)], my_vicon.noisy_quat_rates[:,1], 'g',
        my_vicon.interp_times[:len(my_vicon.smoothed_quat_rates)], my_vicon.smoothed_quat_rates[:,1], '--y',
        my_vicon.interp_times[:len(my_vicon.noisy_quat_rates)], my_vicon.noisy_quat_rates[:,2], 'b',
        my_vicon.interp_times[:len(my_vicon.smoothed_quat_rates)], my_vicon.smoothed_quat_rates[:,2], '--c',
        my_vicon.interp_times[:len(my_vicon.noisy_quat_rates)], my_vicon.noisy_quat_rates[:,3], 'k',
        my_vicon.interp_times[:len(my_vicon.smoothed_quat_rates)], my_vicon.smoothed_quat_rates[:,3], '--r',)


        plt.show()
        '''
