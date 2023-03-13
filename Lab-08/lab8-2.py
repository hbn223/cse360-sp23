import sys
import time
import socket
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1

import math
import numpy as np

positions = {}
rotations = {}

IP_ADDRESS = '192.168.0.203'

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

def orient_control(orient_err):
    omega = 1500 * math.atan2( (math.sin(orient_err)), (math.cos(orient_err)) )
    return omega

def linear_velocity_control(dist):
    v = 1000 * dist
    return v

if __name__ == "__main__":

    clientAddress = "192.168.0.44"
    optitrackServerAddress = "192.168.0.4"

    robot_id = 203

    point = [-2.77, 2.22]

    # rectangular trajectory
    p1 = [-3.0, 2.0]
    p2 = [-2.0, 2.0]
    p3 = [-2.0, 1.5]
    p4 = [-3.0, 1.5]

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
        try:
            if robot_id in positions:
                # last position
                print('Last position', positions[robot_id], ' rotation', rotations[robot_id])
                time.sleep(1)

                robot_orient = math.radians(rotations[robot_id])                    # robot orientation, (rotations) deg --> rad
                robot_pos = [positions[robot_id][0], positions[robot_id][1]]        # robot position
                dist = math.dist(robot_pos, point)                                  # distance to point


                desired_orient = ( math.atan2( ((point[1]-robot_pos[1])) , ((point[0]-robot_pos[0]) ))  )    # desired orientation, rad
                orient_err = desired_orient - robot_orient                          # orientation error
                
                # print('Distance from arbitrary point g = [', point[0], ', ', point[1], ']: ', dist)

                print('Distance from point: \t', dist)
                print('Robot orientation: \t',   robot_orient)
                print('Desired orientation: \t', desired_orient)
                print('Orientation error: \t',   orient_err)
                print('\n')

                v = 1000 * dist
                omega = 1500 * math.atan2( (math.sin(orient_err)), (math.cos(orient_err)) )

                # omega = 0
                # omega = orient_control(orient_err)
                # v = 0
                # v = linear_velocity_control(dist)

                u = np.array([v - omega, v + omega])
                u[u > 1500] = 1500
                u[u < -1500] = -1500
                # Send control input to the motors
                command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(u[0], u[0], u[1], u[1])
                s.send(command.encode('utf-8'))

        except KeyboardInterrupt:
            command = 'CMD_MOTOR#00#00#00#00\n'
            s.send(command.encode('utf-8'))
            break

s.shutdown(2)
s.close()
