import json
import utils
from pyrcareworld.envs.bathing_env import BathingEnv
import numpy as np
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
    random_seed= utils.manikin_randomizer(True)
    print(random_seed)
    env = BathingEnv(graphics=use_graphics, seed=random_seed) if dev == False else BathingEnv(graphics=use_graphics, executable_file="@editor")
    robot = env.get_robot()
    robot = utils.random_robot_pose(robot)
    env.step()
    sponge = env.get_sponge()
    print(sponge.data)
    sponge = utils.random_sponge_pose(sponge)
    env.step()
    print(sponge.data)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run RCareWorld bathing environment simulation.')
    parser.add_argument('-g', '--graphics', action='store_true', help='Enable graphics')
    parser.add_argument('-d', '--dev', action='store_true', help='Run in developer mode')
    args = parser.parse_args()
    _main(use_graphics=args.graphics, dev=args.dev)
