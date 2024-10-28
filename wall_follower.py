import time
import numpy as np
from mbot_bridge.api import MBot


def find_min_dist(ranges, thetas):
    """Finds the length and angle of the minimum ray in the scan.

    Make sure you ignore any rays with length 0! Those are invalid.

    Args:
        ranges (list): The length of each ray in the Lidar scan.
        thetas (list): The angle of each ray in the Lidar scan.

    Returns:
        tuple: The length and angle of the shortest ray in the Lidar scan.
    """

    min_dist, min_angle = None, None
    valid_ranges = np.array(ranges)
    valid_thetas = np.array(thetas)

    valid_index= (valid_ranges > 0).nonzero()[0]
    if len(valid_index)==0:
        return None, None

    min_index = valid_ranges[valid_index].argmin()
    min_dist = valid_ranges[valid_index][min_index]
    min_angle = valid_thetas[valid_index][min_index]

    

    # TODO: Find the length and angle of the shortest distance in the ray.

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
    return res


robot = MBot()
setpoint = 0  # TODO: Pick your setpoint.
# TODO: Declare any other variables you might need here.

try:
    # Read the latest lidar scan.
    ranges, thetas = robot.read_lidar()
    min_dist, min_angle=find_min_dist(ranges,thetas)
    
    for i in range(2):
        print(min_dist,min_angle)
    

        # TODO: (P1.2) Write code to follow the nearest wall here.
        # Hint: You should use the functions cross_product and find_min_dist.

        # Optionally, sleep for a bit before reading a new scan.
        time.sleep(0.1)
except KeyboardInterrupt:
   print("Control+C pressed.")
    # Catch any exception, including the user quitting, and stop the robot.
    robot.stop()
