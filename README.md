# Event Cat
Learned State Estimation
- State Estimation using an Event Camera + IMU

**Data Folder**
We have a total of 18 Rosbags processed and recorded into binary files. Each has 4 files associated:

rosbagXX_vicon - interpolated vicon data. Has dimensions (timestamps(velocity_estimate, quaternion, angular_rate_estimate)) of which:
- Velocity Estimate has dimensions (x_dot, y_dot, z_dot)
- Quaternion has dimensions (i,j,k, scalar)
- Angular Rate Estimate has dimensions (theta_dot, phi_dot, psi_dot)

rosbagXX_imu - IMU data, of dimensions (timestamps(linear_accelerations, gyros)).
- Linear Accelerations has dimensions (x_dotdot, y_dotdot, z_dotdot)
- Gyros has dimensions (theta_dot, phi_dot, psi_dot)

rosbagXX_ec - Event Camera data, of dimension timestamps(event1,..., event75)
- Each event has dimensions (X,Y,Polarity(0 or 1))


Each of the Rosbags are as follows:

*Rosbag 1:*
