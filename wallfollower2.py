import time
import numpy as np
from mbot_bridge.api import MBot
import math

#Done
def find_min_dist(ranges, thetas):
    """Finds the length and angle of the minimum ray in the scan.

    Make sure you ignore any rays with length 0! Those are invalid.

    Args:
        ranges (list): The length of each ray in the Lidar scan.
        thetas (list): The angle of each ray in the Lidar scan.

    Returns:
        tuple: The length and angle of the shortest ray in the Lidar scan.
    """
    valid_rays = []
    for dist, angle in zip(ranges, thetas):
        if dist > 0:
            valid_rays.append((dist, angle))

    if not valid_rays:
        return None, None
    
    valid_distances, valid_angles = [ray[0] for ray in valid_rays], [ray[1] for ray in valid_rays]

    min_dist, min_angle = None, None

    min_dist = min(valid_distances)
    index = valid_distances.index(min_dist)
    min_angle = valid_angles[index]

    return min_dist, min_angle


def cross_product(v1, v2):
    """Compute the Cross Product between two vectors.

    Args:
        v1 (list): First vector of length 3.
        v2 (list): Second vector of length 3.

    Returns:
        list: The result of the cross product operation.
    """
    res = np.zeros(3)
    # TODO: Compute the cross product.
    res = np.cross(v1,v2)
    return res

robot = MBot()
setpoint = 0.1  # TODO: Pick your setpoint.
# TODO: Declare any other variables you might need here.
ranges, thetas = robot.read_lidar()
# print(find_min_dist(ranges, thetas))

try:
    while True:
        # Read the latest lidar scan.
        ranges, thetas = robot.read_lidar()

        # TODO: (P1.2) Write code to follow the nearest wall here.
        # Hint: You should use the functions cross_product and find_min_dist.

        #Step 1 Find the distance to the nearest wall and the angle where the wall is located (use findMinDist() for this part).
        min_dist,min_angle = find_min_dist(ranges,thetas)

        #Step 2 Use the cross product to find a vector pointing parallel to the wall, in the direction the robot should drive.
        mag= 0.5
        v_to_wall = [1*math.cos(min_angle),1*math.sin(min_angle),0]

        follow = mag * cross_product(v_to_wall, [0,0,1])

        #Step 3 Apply a correction to the vector using bang-bang or P-control to move closer to or farther from the wall, depending on the current distance to the wall.
        kp = 0.25
        if min_dist < setpoint:
            error = setpoint - min_dist
            follow += kp*error
        if min_dist > setpoint:
            error = min_dist - setpoint
            follow -= kp*error 
        if min_dist == setpoint:
            pass
        print(follow)
        #Step 4 Convert the vector to a velocity vector and send a velocity command to the robot.
        robot.drive(follow[0], follow[1], follow[2])

        # Optionally, sleep for a bit before reading a new scan.
        time.sleep(0.25)
except:
    # Catch any exception, including the user quitting, and stop the robot.
    robot.stop()
