import sys
import time
import socket
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1

import math
import numpy as np
import cv2
from picamera2 import Picamera2 

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()

positions = {}
rotations = {}

IP_ADDRESS = '192.168.0.206'

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_ADDRESS, 5000))
print('Connected\n')

# coordinates of points along trajectory
x_point = []
y_point = []  


# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles

    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz


def linear_velocity_control(dist):
    kv = 1000
    v = kv * dist
    return v

def angular_velocity_control(alpha, theta):
    kw = 150
    w = kw * math.degrees( math.atan2( math.sin(alpha - theta), math.cos(alpha - theta) ))
    return w

def move(robot_id, position, index):
    while True:
        x_curr = positions[robot_id][0]
        y_curr = positions[robot_id][1]
        robot_pos = [x_curr, y_curr]
        goal = [x_point[i], y_point[i]]
        dist = math.dist(robot_pos, goal)

        robot_orient = math.radians(rotations[robot_id])        # theta
        desired_orient = math.atan2( (goal[1]-robot_pos[1]) , (goal[0]-robot_pos[0]) )      # alpha

        v = 2500 * dist
        omega = 200 * math.degrees( math.atan2( math.sin(desired_orient - robot_orient), math.cos(desired_orient - robot_orient) ))

        u = np.array([v - omega, v + omega])
        u[u > 1500] = 1500
        u[u < -1500] = -1500
        # Send control input to the motors
        command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
        s.send(command.encode('utf-8'))
        time.sleep(.5)

        if (dist < 0.2):
            break   

if __name__ == "__main__":
    try:
        clientAddress = "192.168.0.33"
        optitrackServerAddress = "192.168.0.4"
        robot_id = 206

        # This will create a new NatNet client
        streaming_client = NatNetClient()
        streaming_client.set_client_address(clientAddress)
        streaming_client.set_server_address(optitrackServerAddress)
        streaming_client.set_use_multicast(True)
        # Configure the streaming client to call our rigid body handler on the emulator to send data out.
        streaming_client.rigid_body_listener = receive_rigid_body_frame

        # Start up the streaming client now that the callbacks are set up.
        # This will run perpetually, and operate on a separate thread.
        is_running = streaming_client.run()

        while is_running:
            if robot_id in positions:
                
                # Blob detection
                frame = picam2.capture_array()

                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

                # Threshold of yellow in HSV space
                lower_yellow = np.array([20, 100, 100])
                upper_yellow = np.array([30, 255, 255])
            
                # preparing the mask to overlay
                mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
                
                # Setup SimpleBlobDetector parameters.
                params = cv2.SimpleBlobDetector_Params()
                
                params.filterByArea = True
                params.minArea = 500
                params.filterByCircularity = False
                params.filterByConvexity = False
                params.filterByInertia = False

                # Read image
                img = cv2.imread("blob.jpg", cv2.IMREAD_GRAYSCALE)

                detector = cv2.SimpleBlobDetector_create(params)

                # Detect blobs.
                keypoints = detector.detect(img)

                ##
                # -----------------
                # TO BE IMPLEMENTED
                # -----------------
                #
                # Logic to translate blob detection data of ducks in the environments
                # into distance from the robot. The algorithm will select the biggest
                # blob detected (= closest duck in field of vision). With the distance,
                # relative position, and angle of the robot in relation to the duck and
                # the world frame, calculations can be made to determine the x and y 
                # coordinates of the duck.
                # 
                # Applying the same mechanisms as the midterm project, the array of x & y
                # positions are used to inform the robot's movements to pick up the duck.
                #
                # Assuming there is a fixed location for the rescue circle, once it is
                # determined that a duck has been captured (blob detection returns blob
                # with certain area, height, or width), the coordinates of the rescue
                # circle can be added as a node along the robot's trajectory, guiding it back.
                #
                # -----------------
                #
                # The handling of obstacles in the environment is discussed further in 
                # the lab report. Mapping the whole space of the lab as a grid, a fixed
                # amount of nodes can represent locations along the grid. The strategy 
                # I have in mind is to have the robot traverse the nodes along a path,
                # detecting ducks as it goes, rescue the duck, take the shortest path to
                # travel back to the rescue circle. A for loop is utilized so the robot
                # continues traversing and rescuing ducks, until all nodes within the grid
                # have been covered.
                # 
                # The obstacles can be marked by certain nodes. Then, it is easy to guide
                # the robot to move around obstacles, by instructing it to not traverse
                # these specific nodes (maybe by removing these obstacle nodes from the
                # path).
                #
                ##

                for i in range(len(x_point)):
                    # Feeding positions into function move to get instructions for moving the
                    # robot to the duck/next node in its path. As shown in the midterm, the 
                    # robot can successfully move between a set of pre-determined points in 
                    # the environment.
                    move(robot_id, positions, i)

                break

    except KeyboardInterrupt:
        command = 'CMD_MOTOR#00#00#00#00\n'
        s.send(command.encode('utf-8'))
        sys.exit()

command = 'CMD_MOTOR#00#00#00#00\n'
s.send(command.encode('utf-8'))
print("Close connection")
s.shutdown(2)
s.close()
