import json
import utils
from pyrcareworld.envs.bathing_env import BathingEnv
import bathing_perception as perception
import numpy as np
import math
import cv2
import argparse



def _main(use_graphics=False, dev=None):
    if use_graphics:
        text = """
        An example of the usage of the bathing environment.

        The sponge will be attached to the robot's hand if the grasp center and the sponge are close enough. (distance < 0.1m)  
        The sponge will be detached from the robot's hand if you call the GripperOpen() function.

        You can obtain low level information of the sponge, the robot, and use unlimited numbers of cameras to observe the scene.

        The threshold for a comfortable force on the human body is set to 1-6N.

        Check the website detailed rubric. After each run of the simulation, a json file will be generated in the current directory (~/.config/unity3d/RCareWorld/BathingPlayer).

        The path may be different according to the OS and your computer configuration.
        """

        print(text) 
    # Initialize the environment
    random_seed = utils.manikin_randomizer(True) # Comment for submission
    print(random_seed)
    env = BathingEnv(graphics=use_graphics, seed=random_seed) if dev == False else BathingEnv(graphics=use_graphics, executable_file="@editor")
    robot = env.get_robot()
    sponge = env.get_sponge()

    # read waypoints from json file
    with open('waypoints.json') as f:
        waypoints_data = json.load(f)
        print(waypoints_data)
    
    print(robot.data['position'])
    print(robot.data['rotation'])

    # random robot pose and sponge. Comment this section for submission! 
    robot = utils.random_robot_pose(robot)
    sponge = utils.random_sponge_pose(sponge)
    env.step(100) #waiting for robot and sponge to stabilise
    #----------------------------------------------#


    # loading grasping position for sponge
    grasping_position = waypoints_data['grasping']['position']
    grasping_rotation = waypoints_data['grasping']['rotation']
    #env.Pend() to record videos

    # calculating orientation and distance
    direction, rot, dist = utils.move(robot.data['position'], grasping_position, robot.data['rotation'][1])

    print("direction: ", direction)
    print("rot: ", rot)
    print("dist: ", dist)

    if direction == "Left" :
        robot.TurnLeft(rot, 1)
    else:
        robot.TurnRight (rot, 1)

    # performing rotation action with required time step
    env.step(utils.calculate_step_rotation(rot)) 

    # moving forward
    robot.MoveForward(dist, 1)
    env.step(utils.calculate_step_translation(dist))

    # performing rotation action with required time step
    direction, rot  = utils.rotate(robot.data['rotation'][1], grasping_rotation)
    if direction == "Left" :
        robot.TurnLeft(rot, 1)
    else:
        robot.TurnRight (rot, 1)
    env.step(utils.calculate_step_rotation(rot))


    print(robot.data['position'])
    print(robot.data['rotation'])

    env.Pend()




if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run RCareWorld bathing environment simulation.')
    parser.add_argument('-g', '--graphics', action='store_true', help='Enable graphics')
    parser.add_argument('-d', '--dev', action='store_true', help='Run in developer mode')
    args = parser.parse_args()
    _main(use_graphics=args.graphics, dev=args.dev)
