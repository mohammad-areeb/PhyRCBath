import argparse
from pyrcareworld.envs.bathing_env import BathingEnv
import bathing_perception as perception
import numpy as np
import cv2

def _main(use_graphics=False): 
    # Initialize the environment
    env = BathingEnv(graphics=use_graphics)

    robot = env.get_robot()
    env.step()
    print(robot.data)

    # Control the gripper
    gripper = env.get_gripper()
    gripper.GripperOpen()
    env.step(300)

    gripper.GripperClose()
    env.step(300)

    #create a new perception object and pass in the environment variable
    p = perception.Perception(env)

    #get coordinates of sponge
    print("Sponge coords: ", p.get_sponge_coords())

    #get coordinates of gripper
    print("Gripper coords: ", p.get_gripper_coords())

    #get coordinates of centre of water tank
    print("Water tank coords: ", p.get_water_tank())

    # IMPORTANT you must call generate_landmarks before you can access them!
    p.generate_landmarks()
    
    #query the manikin_landmarks array for a specific landmark by name
    print("Coords of manikin's right elbow: ", p.get_manikin(body_part="RIGHT_ELBOW"))
    #query the manikin_landmarks array for a specific landmark by number
    print("Coords of manikin's landmark 14: ", p.get_manikin(landmark_num=14))
    env.step()

    #height of elbow from floor
    print(p.get_depth(14))
    # height of nose from floor
    print(p.get_depth(0))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run RCareWorld bathing environment simulation.')
    parser.add_argument('-g', '--graphics', action='store_true', help='Enable graphics')
    args = parser.parse_args()
    _main(use_graphics=args.graphics)
