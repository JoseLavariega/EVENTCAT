
from EventCameraProcessing import EventCameraPreprocessor
import numpy as np
import matplotlib.pyplot as plt
import rosbag
import time
import math

class IMUPreprocessor:
    def __init__(self):
        self.IMU_MOVIE  = []
        self.START_TIME = 0.0
        

        #initialialization
        self.linear_x = []
        self.linear_y = []
        self.linear_z = []

        self.gyro_x   = []
        self.gyro_y   = []
        self.gyro_z   = []

        self.times    = []

    def rosbag_populate_network_imu(self, bag, FRAMERATE):
        counter = 0
        PREVIOUS_FRAME = 1
        RATE_HERTZ = 1./FRAMERATE

        for topic, msg, t in bag.read_messages(topics=['/dvs/imu']):
            if(PREVIOUS_FRAME >= 100):
               break

            if (counter == 0):
                ROSBAG_START = msg.header.stamp.secs + msg.header.stamp.nsecs*0.000000001
                previous_frame_time = ROSBAG_START
                counter += 1
            
            current_time = msg.header.stamp.secs + msg.header.stamp.nsecs*0.000000001
            rosbag_time = current_time - ROSBAG_START

            if (rosbag_time >= PREVIOUS_FRAME * RATE_HERTZ):
                lin_x = msg.linear_acceleration.x
                lin_y = msg.linear_acceleration.y
                lin_z = msg.linear_acceleration.z
                
                gyr_x = msg.angular_velocity.x
                gyr_y = msg.angular_velocity.y
                gyr_z = msg.angular_velocity.z


                self.linear_x.append(lin_x)
                self.linear_y.append(lin_y)
                self.linear_z.append(lin_z)

                self.gyro_x.append(gyr_x)
                self.gyro_y.append(gyr_y)
                self.gyro_z.append(gyr_z)

                self.times.append(rosbag_time)

                self.IMU_MOVIE.append([lin_x, lin_y, lin_z, gyr_x, gyr_y, gyr_z])
                print(PREVIOUS_FRAME)
                PREVIOUS_FRAME += 1


if __name__ == '__main__':
    bag = rosbag.Bag('/home/joselavariega/bagfiles/test_7_jumps3.bag')
    my_imu = IMUPreprocessor()
    FRAMERATE = 500

    start_time = time.perf_counter()
    my_imu.rosbag_populate_network_imu(bag, FRAMERATE)
    end_time   = time.perf_counter()
    
    bag.close()
    print(end_time - start_time)

    print(len(my_imu.IMU_MOVIE))
    print(len(my_imu.IMU_MOVIE[:][0]))
    print(my_imu.IMU_MOVIE[0])

    fig = plt.figure()
    ax1 = fig.add_subplot(221)
    ax2 = fig.add_subplot(222)

    ax1.plot(my_imu.times, my_imu.linear_x, 'r', my_imu.times, my_imu.linear_y, 'g',
            my_imu.times, my_imu.linear_z, 'b')

    ax2.plot(my_imu.times, my_imu.gyro_x, 'r', my_imu.times, my_imu.gyro_y, 'g' , 
            my_imu.times, my_imu.gyro_z, 'b')

    ax1.legend(['lin_acc x', 'lin_acc y', 'lin_acc z'])
    ax2.legend(['gyro x', 'gyro y', 'gyro z'])

    print(my_imu.IMU_MOVIE[:][:][0])

    IMU_nparray = np.array(my_imu.IMU_MOVIE)

    print(IMU_nparray[:,0])

    #
    # print(len(my_imu.IMU_MOVIE[:,0]))

    input('Enter to Continue!')
     
    ax3 = fig.add_subplot(223)
    ax4 = fig.add_subplot(224)

    ax3.plot(my_imu.times, IMU_nparray[:,0], 'r', my_imu.times, IMU_nparray[:,1], 'g',
            my_imu.times, IMU_nparray[:,2], 'b')


    ax4.plot(my_imu.times, IMU_nparray[:,3], 'r',my_imu.times, IMU_nparray[:,4], 'g',
            my_imu.times, IMU_nparray[:,5], 'b')

    ax3.legend(['lin_acc x', 'lin_acc y', 'lin_acc z'])
    ax4.legend(['gyro x', 'gyro y', 'gyro z'])


    plt.show()

