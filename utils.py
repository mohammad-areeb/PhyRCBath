import numpy as np


def manikin_randomizer(randomize = False): 
    """
    Return a random seed to randomize the manikin pose
    :param randomize:  Bool
    :return: int with seed value if any
    """
    if(randomize == False):
        return None
    else:
        return np.random.randint(0, 100) #random pose of the manikin


def random_robot_pose(robot): 
    """
    Set a random robot position [+- 25 cm] and orientation [+- 50 deg]
    :param robot: robot object
    :return: robot object with updated position and rotation
    """
    #random starting position of the robot within a square of 0.5x0.5
    randomX = np.random.uniform(0, 1)*0.5 - 0.25
    randomZ = np.random.uniform(0, 1)*0.5  - 0.25
    randomR = np.random.randint(0, 100) - 50
    robot_pos = robot.data['position']
    robot_orientation = robot.data['rotation']
    new_position = [robot_pos[0]+randomX, robot_pos[1], robot_pos[2]+randomZ]
    new_rotation = [robot_orientation[0], robot_orientation[1]+randomR, robot_orientation[2]]
    robot.SetPosition(new_position, True)
    robot.SetRotation(new_rotation, True)
    return robot

def random_sponge_pose(sponge): 
    """
    Set a random robot position [+- 10 cm] and orientation [+- 45 deg]
    :param robot: robot object
    :return: robot object with updated position and rotation
    """
    randomX = np.random.uniform(0, 1)*0.1 - 0.05
    randomZ = np.random.uniform(0, 1)*0.1  - 0.05
    randomR = np.random.randint(0, 90) - 45
    sponge_pos = sponge.data['position']
    sponge_orientation = sponge.data['rotation']
    new_position = [sponge_pos[0]+randomX, sponge_pos[1], sponge_pos[2]+randomZ]
    new_rotation = [sponge_orientation[0], sponge_orientation[1]+randomR, sponge_orientation[2]]
    sponge.SetPosition(new_position, True)
    sponge.SetRotation(new_rotation, True)
    return sponge
