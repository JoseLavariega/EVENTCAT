import pandas as pd
import numpy as np 
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import csv
import cv2 #open cv
import rosbag
import rospy
import time
import pickle as pk
from inspect import EndOfBlock
from queue import Queue

# ros imports here. make a topic list

#create subscriber, create publisher, create nodes?
class EventCameraPreprocessor:
    def __init__(self):
        self.NATIVE_EVENTS_MOVIE = []
        self.CORNER_EVENTS_MOVIE = []
        self.NETWORK_DATA_CORNER_EVENTS = []
        self.TIMESTAMPS_PER_FRAME = 500
        self.SENSOR_HEIGHT = 260
        self.SENSOR_WIDTH  = 346
        self.START_TIME    = 0.0   # or None

        self.EVENTS_IN_FRAME = 75

        # helper 
        self.EVENTS_PER_FRAME =  [] #used to compute the average of all frames at different rates


        self.circle3 = [[0,3], [1,3], [2,2], [3,1],  
                [3,0], [3,-1], [2,-2],[1,-3],
                [0,-3],[-1,-3],[-2,-2],[-3,-1],
                [-3,0], [-3,1], [-2,2],[-1,3]]

        self.circle4 = [[0,4], [1,4], [2,3], [3,2], [4,1], 
                [4,0], [4,-1], [3,-2], [2,-3], [1,-4],
                [0,-4], [-1,-4], [-2,-3], [-3,-2], [-4,-1],
                [-4,0], [-4,1], [-3,2], [-2,3], [-1,4]]

    #initialization
        self.sae_ = [np.zeros((self.SENSOR_WIDTH, self.SENSOR_HEIGHT)), np.zeros((self.SENSOR_WIDTH, self.SENSOR_HEIGHT))]
        self.sae_[0] = np.zeros((self.SENSOR_WIDTH, self.SENSOR_HEIGHT))
        self.sae_[1] = np.zeros((self.SENSOR_WIDTH, self.SENSOR_HEIGHT))

        #print(np.shape(self.sae_[0]))

        #corner events
        self.corner_x_pos = []
        self.corner_x_neg = []
        self.corner_y_pos = []
        self.corner_y_neg = []

        #all events
        self.event_x_pos = []
        self.event_x_neg = []
        self.event_y_pos = []
        self.event_y_neg = []

        self.network_corner_events = []
        self.throwaway_frame  = np.zeros

        
    def populate_network_corner_frames(self, event):
        if (self.is_feature(event)):
            size = len(self.network_corner_events)
            if(size>=75):
                self.network_corner_events.pop(0)
                self.network_corner_events.append([event.x, event.y, event.polarity])
                #print('I Have popped!')
            else:
                self.network_corner_events.append([event.x, event.y, event.polarity])
            
    def populate_frames(self, event):
        if(event.polarity): # polarity is True/ False
            self.event_x_pos.append(event.x)
            self.event_y_pos.append(event.y)

        else:
            self.event_x_neg.append(event.x)
            self.event_y_neg.append(event.y)
    
    def populate_corner_frames(self, event):
        
        if (self.is_feature(event)):
            x = 1
            #want to process it first. 
            if(event.polarity):  # polarity is True/False
                self.corner_x_pos.append(event.x)
                self.corner_y_pos.append(event.y)
            else:
                self.corner_x_neg.append(event.x)
                self.corner_y_neg.append(event.y)

    def is_feature(self, event): # FAST ALGORITHM
        #print('Entered Is Feature without issue')
        polarity = 1 if event.polarity == True else 0 #reverify with rosbag
        self.sae_[polarity][event.x, event.y] = event.ts.secs + event.ts.nsecs*0.000000001

        max_scale = 1

        #discard if too close to the border:
        cs = max_scale*4 # for a circle of radius 4 maximum distance
        if((event.x < cs or event.x >= self.SENSOR_WIDTH-cs) or (event.y<cs or event.y >= self.SENSOR_HEIGHT-cs)):
            return False
        
        found_streak = False

        for i in range(0,16):
            for streak_size in range(3,7):
                # Check that streak event is larger than neighbour

                current_point_timeval = self.sae_[polarity][ event.x+self.circle3[i][0], event.y+self.circle3[i][1] ] 
                neighbor_point_timeval= self.sae_[polarity][ event.x+self.circle3[(i-1+16)%16][0], event.y+self.circle3[(i-1+16%16)][1] ]

                if(current_point_timeval < neighbor_point_timeval):
                    continue

                
                #Check that streak event is larger than the neighbor
                current_streaksize_timeval = self.sae_[polarity][ event.x+self.circle3[(i+streak_size-1)%16][0], event.y+self.circle3[(i+streak_size-1)%16][1] ]
                neighbor_streaksize_timeval= self.sae_[polarity][ event.x+self.circle3[(i+streak_size)%16][0], event.y+self.circle3[(i+streak_size)%16][1] ]
                
                if(current_streaksize_timeval < neighbor_streaksize_timeval):
                    continue

                min_t = self.sae_[polarity][event.x+self.circle3[i][0], event.y+self.circle3[i][1]]/1.0

                for j in range(1,streak_size):
                    tj = self.sae_[polarity][ event.x+self.circle3[(i+j)%16][0] , event.y+self.circle3[(i+j)%16][1] ]/1.0
                    if(tj<min_t):
                        min_t = tj
                
                did_break = False
                for j in range(streak_size, 16):
                    tj = self.sae_[polarity][ event.x+self.circle3[(i+j)%16][0] , event.y+self.circle3[(i+j)%16][1] ]/1.0

                    if(tj >= min_t):
                        did_break = True
                        break
                
                if(not did_break):
                    found_streak = True
                    break
            
            if found_streak:
                break
        

        if found_streak:
            found_streak = False
            
            for i in range(0,20):
                for streak_size in range(4,9):
                
                    current_point_timeval = self.sae_[polarity][ event.x+self.circle4[i][0], event.y+self.circle4[i][1] ] 
                    neighbor_point_timeval= self.sae_[polarity][ event.x+self.circle4[(i-1+20)%20][0], event.y+self.circle4[(i-1+20%20)][1] ]

                    if(current_point_timeval < neighbor_point_timeval):
                        continue

                    current_streaksize_timeval = self.sae_[polarity][ event.x+self.circle4[(i+streak_size-1)%20][0], event.y+self.circle4[(i+streak_size-1)%20][1] ]
                    neighbor_streaksize_timeval= self.sae_[polarity][ event.x+self.circle4[(i+streak_size)%20][0], event.y+self.circle4[(i+streak_size)%20][1] ]

                    if (current_streaksize_timeval < neighbor_streaksize_timeval):
                        continue

                    min_t = self.sae_[polarity][event.x+self.circle4[i][0], event.y+self.circle4[i][1]]/1.0

                    for j in range(1,streak_size):
                        tj = self.sae_[polarity][ event.x+self.circle4[(i+j)%20][0] , event.y+self.circle4[(i+j)%20][1] ]/1.0

                        if (tj < min_t):
                            min_t = tj
                    
                    did_break = False

                    for j in range(streak_size,16):
                        tj = self.sae_[polarity][ event.x+self.circle4[(i+j)%20][0] , event.y+self.circle4[(i+j)%20][1] ]/1.0

                        if(tj >= min_t):
                            did_break = True
                            break

                    if(not did_break):
                        found_streak = True
                        break
                
                if found_streak:
                    break
        #print('Exit Is feature without issue')
        return found_streak
    
    def rosbag_populate_movie_frames(self, bag, FRAMES_PER_SECOND):
        counter = 0
        PREVIOUS_FRAME = 1
        RATE_HERTZ = 1./FRAMES_PER_SECOND

        for topic, msg, t in bag.read_messages(topics=['/dvs/events']):
            if(PREVIOUS_FRAME >= 250):
               break

            if (counter == 0):
                ROSBAG_START = msg.header.stamp.secs+msg.header.stamp.nsecs*0.000000001
                previous_frame_time = ROSBAG_START    
                counter += 1

            current_time = msg.header.stamp.secs+ msg.header.stamp.nsecs*0.000000001
            rosbag_time_out  = current_time - ROSBAG_START

            for event_msg in msg.events:
            #build frame or next frame
                event_time = event_msg.ts.secs + event_msg.ts.nsecs*0.000000001
                rosbag_time = event_time - ROSBAG_START

            
                if(rosbag_time <= PREVIOUS_FRAME*RATE_HERTZ):
                    # add to current frame
                    self.populate_frames(event_msg)
                    self.populate_corner_frames(event_msg)
                

                else: 
                    PREVIOUS_FRAME += 1
                    print(PREVIOUS_FRAME*RATE_HERTZ)
                    events_in_frame = len(self.corner_x_neg) + len(self.corner_x_pos)
                    
                    #self.EVENTS_PER_FRAME.append(events_in_frame)

                    #print(events_in_frame) # different sparcities in negative or positive events
                    

                    # add in the frame, Frame is done
                    self.NATIVE_EVENTS_MOVIE.append([self.event_x_pos, self.event_y_pos,
                                                            self.event_x_neg, self.event_y_neg])

                    self.CORNER_EVENTS_MOVIE.append([self.corner_x_pos, self.corner_y_pos,
                                                                self.corner_x_neg, self.corner_y_neg])

                    #unpopulate the frames
                    self.event_x_neg = []
                    self.event_x_pos = []
                    self.event_y_neg = []
                    self.event_y_pos = []

                    self.corner_x_neg = []
                    self.corner_x_pos = []
                    self.corner_y_neg = []
                    self.corner_y_pos = []
 
    def rosbag_populate_network_frames(self, bag, FRAMES_PER_SECOND):
        # want to populate each frame with EVENTS_IN_FRAME events, regardless of size
        local_counter_events = 0
        counter = 0
        PREVIOUS_FRAME = 1
        RATE_HERTZ = 1./FRAMES_PER_SECOND


        for topic, msg, t in bag.read_messages(topics=['/dvs/events']):
            if(PREVIOUS_FRAME >= 750):
                break

            if (counter == 0): # Assign an initial timestamp to the bag
                ROSBAG_START = msg.header.stamp.secs+msg.header.stamp.nsecs*0.000000001
                previous_frame_time = ROSBAG_START    
                counter += 1

            current_time = msg.header.stamp.secs+ msg.header.stamp.nsecs*0.000000001
            rosbag_time_out  = current_time - ROSBAG_START

            for event in msg.events:
                event_time = event.ts.secs + event.ts.nsecs*0.000000001
                rosbag_time = event_time - ROSBAG_START

                self.populate_network_corner_frames(event) #no matter what always populate if a suitable event 
                
                

                if(rosbag_time > PREVIOUS_FRAME*RATE_HERTZ):
                    #print(PREVIOUS_FRAME)
                    #print(PREVIOUS_FRAME*RATE_HERTZ)

                    events_in_frame = len(self.network_corner_events)
                    self.EVENTS_PER_FRAME.append(events_in_frame)
                    finished_frame = self.network_corner_events.copy()
                    self.CORNER_EVENTS_MOVIE.append(finished_frame)

                    #print(events_in_frame)
                    #print(self.network_corner_events)
                    #if(events_in_frame == self.EVENTS_IN_FRAME):
                    #print(self.CORNER_EVENTS_MOVIE)

                    PREVIOUS_FRAME += 1
                    
                    
        #print(self.CORNER_EVENTS_MOVIE)
        #print(self.CORNER_EVENTS_MOVIE[1])
        #print(self.CORNER_EVENTS_MOVIE[100])
        bag.close()

        

                 

if __name__ == '__main__':
    bag = rosbag.Bag('/home/joselavariega/bagfiles/test_7_jumps3.bag')
    counter = 0
    PREVIOUS_FRAME = 1
    my_EventCamera = EventCameraPreprocessor()
    FRAMES_SECOND = 500 #0.03 is 30 Hz. 0.025 is 40 Hz. 

    # parameters to note 
    # 0.03  - 30   Hz
    # 0.025 - 40   Hz
    # 0.0167- 60   Hz
    # 0.01  - 100  Hz
    # 0.005 - 200  Hz
    # 0.002 - 500  Hz
    # 0.001 - 1000 

    start_time = time.perf_counter()

    my_EventCamera.rosbag_populate_network_frames(bag,FRAMES_SECOND) 
    #my_EventCamera.rosbag_populate_movie_frames(bag,500)
 
    end_time = time.perf_counter()

    bag.close()

    print(end_time - start_time)
    print('We are done!')

    print('Average Events per Frame:')
    #
    #print(sum(my_EventCamera.EVENTS_PER_FRAME)/len(my_EventCamera.EVENTS_PER_FRAME))

    input('Press Enter to Continue')

    #Save to binary files
    #pk.dump(my_EventCamera.NATIVE_EVENTS_MOVIE, open('NativeEvents_test7jumps3_facedown_500hz','wb'))
    pk.dump(my_EventCamera.CORNER_EVENTS_MOVIE, open('throwaway1','wb')) # CORNER EVENTS MOVIE IS THE FORMAT FOR THE NEURAL NET

    #Visualize the
    fig = plt.figure()
    fig.set_figheight(5)
    fig.set_figwidth(12)

    ax1 = fig.add_subplot(121)
    #ax2 = fig.add_subplot(122)
    plt.gca().invert_yaxis()

    ax1.axis([0,350,0,270])
    #ax2.axis([0,350,0,270])
    VISUALIZING_MOVIE = []
    corner_x_pos = []
    corner_y_pos = []
    corner_x_neg = []
    corner_y_neg = []

    print(len(my_EventCamera.CORNER_EVENTS_MOVIE))
    print(len(my_EventCamera.CORNER_EVENTS_MOVIE[0]))



    #Process Event Camera into Visible Frames
    for i in range(0,len(my_EventCamera.CORNER_EVENTS_MOVIE)):
        seventy_five = my_EventCamera.CORNER_EVENTS_MOVIE[i]

        
        for event in seventy_five: #should run this exactly 75 y

            if(event[2] == True):
                corner_x_pos.append(event[0])
                corner_y_pos.append(event[1])
            else:

                corner_x_neg.append(event[0])
                corner_y_neg.append(event[1])

        VISUALIZING_MOVIE.append([corner_x_pos, corner_y_pos, corner_x_neg, corner_y_neg])
        corner_x_pos = []
        corner_y_pos = []
        corner_x_neg = []
        corner_y_neg = [] # empty them for the next frame


    #VISUALIZING_MOVIE = my_EventCamera.CORNER_EVENTS_MOVIE // if using the other method

    print(len(VISUALIZING_MOVIE))       # should have same number of frames as 
    print(len(VISUALIZING_MOVIE[1]))    # a frame should have length 4
    print(len(VISUALIZING_MOVIE[1][0])) # number of x polarity + datapoints
    print(len(VISUALIZING_MOVIE[1][1])) # number of y polarity - datapoints should be same as above

    
    

    for i in range(len(VISUALIZING_MOVIE)):
        ax1.axis([0,350,270,0])
        #ax2.axis([0,350,270,0])
        #frame = my_EventCamera.NATIVE_EVENTS_MOVIE[i]
        cnr_frame = VISUALIZING_MOVIE[i]
        # build up data 
        #ax2.scatter(frame[0], frame[1], color = 'red', s=3)
        #ax2.scatter(frame[2], frame[3], color = 'black', s=3)

        ax1.scatter(cnr_frame[0], cnr_frame[1], color = 'red', s=2)
        ax1.scatter(cnr_frame[2], cnr_frame[3], color = 'black', s=2)

        plt.pause(0.1)

        ax1.clear()
        
        #ax2.clear()
    plt.show()

