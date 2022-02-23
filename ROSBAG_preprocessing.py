import numpy as np
import matplotlib.pyplot as plt
import rosbag
import time
import math

from EventCameraProcessing import EventCameraPreprocessor
from ViconPreprocessing import ViconPreprocessor
from IMUProcessing import IMUPreprocessor


if __name__ == '__main__':
    bag = rosbag.Bag('/home/joselavariega/bagfiles/test_1_facing_left.bag')
    my_ec    = EventCameraPreprocessor()
    my_imu   = IMUPreprocessor()
    my_vicon = ViconPreprocessor()
    
    FRAMERATE = 500
    my_imu.rosbag_populate_network_imu(bag,FRAMERATE)
    #my_ec.rosbag_populate_network_frames(bag,FRAMERATE)
    my_vicon.vicon_rosbag_data(bag,FRAMERATE)
    
    bag.close()
    my_vicon.build_interpolation_dataset(my_imu.times)

    my_vicon.velocity_estimator()
    my_vicon.ang_rate_estimator()


    print('Done!')


    #Actually build up the thing
    #print(len(my_ec.CORNER_EVENTS_MOVIE))
    print(len(my_imu.IMU_MOVIE))
    print(len(my_vicon.VICON_MOVIE))

    #print('points')
    #print(my_vicon.interp_points)
    #print('quats')
    #print(my_vicon.interp_quat)
    #print('times')
    #print(my_vicon.interp_times)

    #print(my_imu.times)


    # plotting
    #print(my_vicon.est_velocities_local_differentiation)
    #print(str(my_vicon.est_velocities_local_differentiation.size))

    #print(my_vicon.ros_times)

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
