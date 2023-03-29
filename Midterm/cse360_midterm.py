import sys
import time
import socket
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1

import math
import numpy as np

positions = {}
rotations = {}

# coordinates of points along trajectory
# x_point = [1.022, -0.207, -0.212, 0.99, 1.022]
# y_point = [1.263, 1.31, 0.076, 0.038, 1.263]    # start, 1, 2, duck, 1

x_point = [0.432, -0.207, -0.212, 0.99, 1.022, 0.432]
y_point = [1.299, 1.31, 0.076, 0.038, 1.263, 1.299]  

IP_ADDRESS = '192.168.0.206'

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((IP_ADDRESS, 5000))
print('Connected\n')

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

        print('Distance from point: \t', dist)
        print('Theta: \t',   robot_orient)
        print('Alpha: \t', desired_orient)
        print('\n')

        # v = linear_velocity_control(dist)
        # omega = angular_velocity_control(desired_orient, robot_orient)

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
                # last position
                print('Last position', positions[robot_id], ' rotation', rotations[robot_id])
                time.sleep(1)
                for i in range(len(x_point)):
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
