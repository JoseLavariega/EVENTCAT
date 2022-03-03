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

rosbagXX_full - The data from rosbagXX_vicon, rosbagXX_imu, and rosbagXX_ec together by timestamp. Dimensions are preserved. 


Each of the Rosbags are as follows:

*Rosbag 1:* A static recording of the Event Camera placed on a tripod on a table and facing the Backwards direction. 

*Rosbag 2:* A static recording of the Event Camera, handheld from the tripod, facing down at the floor. 

*Rosbag 3:* A static recording of the Event Camera, placed on a tripod on a table and facing the Frontwards direction. 

*Rosbag 4:* A static recording of the Event Cameara, placed on a tripod on a table and facing the Right direction. 

*Rosbag 5:* A controlled dynamic recording of the Event Camera, held from a tripod while facing down at the floor at an angle. The dynamic motion is walking in a straight line, without any turns. 

*Rosbag 6:* A controlled dynamic recording of the Event Camera, held from a tripod while facing down at the floor at an angle. The dynamic motion is walking in a straight line, with turns. (Faces front and back)

*Rosbag 7:* An uncontrolled dynamic recording of the Event Camera, held from a tripod while facing down. Walking in a random pattern along the motion capture room. Event camera turns with the motion. 

*Rosbag 8:* A controlled dynamic recording of the Event Camera, held form a tripod while facing down at an angle. The motion is up and down movement in a line from a high location to a low location. 

*Rosbag 9:* A controlled dynamic recording of the Event Camera, held from a tripod while facing down at an angle. The motion is circular contours in a vertical plane. 

*Rosbag 10:* A controlled dynamic recording of the Event Camera, held from a tripod while facing down at an angle. The motion is circular contours in a vertical plane. 

*Rosbag 11:* An uncontrolled dynamic recording of the Event Camera, held from a tripod while facing down at an angle. The motion is jumps starting from an approximate same point. 


*Rosbag 12:* An uncontrolled dynamic recording of the Event Camera, held from a tripod while facing down at an angle. The motion is jumps along the entire floor of the motion capture room. 'Leapfrogging'

*Rosbag 13:* An uncontrolled dynamic recording of the Event Camera, held from a tripod while facing down at an angle. The motion is jogging in a random motion along the floor of the motion capture room. 

*Rosbag 14:* A controlled dynamic recording of the Event Camera, held from a tripod while facing down at an angle, being subjected to impulses by way of taps and aggressive taps, but without any further movement. 

*Rosbag 15:* A controlled dynamic recording of the Event Camera, held from a tripod while facing down at an angle, being subjected to impulses by way of taps and aggressive taps, but without any further movement. 

*Rosbag 16:* A controlled dynamic recording of the Event Camera, held from a tripod while facing down at an angle, being subjected to impulses by way of taps and aggressive taps, but without any further movement. 

*Rosbag 17:* A controlled dynamic recording of the Event Camera, held from a tripod while facing down at an angle, being subjected to impulses by way of taps and aggressive taps, but without any further movement. 

*Rosbag 18:* A controlled dynamic recording of the Event Camera, held from a tripod while facing down at an angle, being subjected to impulses by way of taps and aggressive taps, but without any further movement. 
