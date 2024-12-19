import json
import utils
from pyrcareworld.envs.bathing_env import BathingEnv
import bathing_perception as perception
import numpy as np
import math
import cv2
import argparse

scaling_fac = 5.546

def compute_distance_3d(fixed_point, changing_point):
    """
    Compute the distance between two points in a 3D space.

    Parameters:
        fixed_point (list or tuple): Coordinates of the fixed point [x1, y1, z1].
        changing_point (list or tuple): Coordinates of the changing point [x2, y2, z2].
    
    Returns:
        float: Distance between the two points.
    """
    # Unpack coordinates
    x1, y1, z1 = fixed_point
    x2, y2, z2 = changing_point

    # Calculate Euclidean distance
    distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
    return distance

def movement_from_A_to_landmark(env, robot, fixed_point, final_point):
    """
    Move the robot from a fixed point to a final point.

    Parameters:
        env (BathingEnv): The simulation environment.
        robot: The robot object.
        fixed_point (list or tuple): Starting coordinates [x1, y1, z1].
        final_point (list or tuple): Target coordinates [x2, y2, z2].
    """
    mid_x = 1.9
    x1, y1, z1 = fixed_point
    trans_point = (mid_x, y1, z1)
    x2, y2, z2 = final_point

    # Move to transitional point
    distance = compute_distance_3d(fixed_point, trans_point)
    print(f"Move back by {distance * scaling_fac} units")
    robot.MoveBack(distance * scaling_fac, 0.5)
    env.step(500)

    # Turn towards the final point
    if z2 < z1:
        print("Turn left")
        robot.TurnRight(90, 1)
    else:
        print("Turn right")
        robot.TurnLeft(90, 1)
    env.step(300)

    # Move to transitional point
    robot_position = robot.data["position"]
    trans_point=(mid_x,y2,z2)
    distance = compute_distance_3d(robot_position, trans_point)
    print(f"Move forward by {distance * scaling_fac} units")
    robot.MoveForward(distance * scaling_fac, 0.5)
    env.step(700)

    # Turn towards the final position
    if x2 < mid_x:
        print("Turn left")
        robot.TurnRight(90, 1)
    else:
        print("Turn right")
        robot.TurnLeft(90, 1)
    env.step(300)

    # Move to final point
    robot_position = robot.data["position"]
    distance = compute_distance_3d(robot_position, final_point)
    print(f"Move forward by {distance * scaling_fac} units")
    robot.MoveForward(distance * scaling_fac, 0.5)
    env.step(400)

    # Stop
    print("Stop")
    robot.TurnRight(90, 1)
    env.step(300)

    print(f"Reached position: {final_point}")

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
    # MOVE TO PICK UP SPONGE POSITION 

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

    #----------------------------------------------#
    # INITIALISING PERCEPTION MODULE

    p = perception.Perception(env)
    env.step(100) # wait for cameras to initialise
    water_tank = p.get_water_tank()
    p.generate_landmarks() # generate mediapipe landmarks

    #----------------------------------------------#
    # PICK UP SPONGE AND DIP IN WATER

    # First, raise the gripper to a safe height to avoid obstacles like the chest
    lift_gripper_position = [robot.data["position"][0], sponge.data["position"][1] + 0.15, robot.data["position"][2]]
    print(f"Raising gripper to safe height: {lift_gripper_position}")
    robot.IKTargetDoMove(
        position=lift_gripper_position,
        duration=2,
        speed_based=False,
    )
    robot.WaitDo()
    env.step(300)

    # Use IK to position the gripper directly above the sponge (adjusting Z direction)
    fine_tune_position = [sponge.data["position"][0], sponge.data["position"][1] + 0.2, sponge.data["position"][2]]  # Position above the sponge at safe height
    print(f"Using IK to position above the sponge: {fine_tune_position}")
    robot.IKTargetDoMove(
        position=fine_tune_position,
        duration=2,
        speed_based=False,
    )
    robot.WaitDo()
    env.step(100)

    # Lower the gripper to reach the sponge
    lower_position = [sponge.data["position"][0], sponge.data["position"][1]+0.05, sponge.data["position"][2]]
    print(f"Lowering gripper to reach sponge: {lower_position}")
    robot.IKTargetDoMove(
        position=lower_position,  # Move directly above the sponge to grasp it
        duration=1,
        speed_based=False,
    )
    robot.WaitDo()
    env.step(100)

    # Control the gripper to grasp the sponge
    gripper = env.get_gripper()
    gripper.GripperClose()
    env.step(300)


    # Rise the gripper to a safe height to avoid obstacles like the chest
    lift_gripper_position = [robot.data["position"][0], sponge.data["position"][1] + 0.15, robot.data["position"][2]]
    print(f"Raising gripper to safe height: {lift_gripper_position}")
    robot.IKTargetDoMove(
        position=lift_gripper_position,
        duration=2,
        speed_based=False,
    )
    env.step(300)

    #Move to dip water
    print(f"Move to water")
    robot.MoveBack(0.25, 0.2)
    env.step(300)

    #print(f"Sponge position: {sponge.data['position']}")

    # Move the robot above the water
    above_water_position = [water_tank[0] + 0.1, 0.8, water_tank[2] - 0.1]
    print(f"Move above water{above_water_position}")
    robot.IKTargetDoMove(
        position=above_water_position,  
        duration=1,
        speed_based=False,
    )
    env.step(300)

    # Lower the gripper to dip the water
    water_position = [water_tank[0] + 0.1, 0.58, water_tank[2] - 0.1]
    print(f"Lowering gripper to dip water {water_position}")
    robot.IKTargetDoMove(
        position=water_position,  
        duration=1,
        speed_based=False,
    )
    env.step(300)


    # Move above water
    print(f"Move above water{above_water_position}")
    robot.IKTargetDoMove(
        position=above_water_position,  
        duration=1,
        speed_based=False,
    )
    env.step(300)

    # Rise the gripper back to a safe position
    lift_gripper_position = [robot.data["position"][0], 1.1, robot.data["position"][2]]
    print(f"Lift back to safe position: {lift_gripper_position}")
    robot.IKTargetDoMove(
        position=lift_gripper_position,
        duration=2,
        speed_based=False,
    )
    env.step(300)

    #----------------------------------------------#
    # MOVE ROBOT TO BATHING START POSITION

    robot_position_prev = robot.data["position"]
    print(f"Robot position: {robot_position_prev}")
    env.step(200)
    # Define start and target positions
    start_position = robot.data["position"]
    target_position = [0.29251301288604736, 0.2508874237537384, 1.3711309432983398+0.5]

    # target_position = p.get_manikin(body_part="LEFT_ELBOW")
    print(f"Target position: {target_position}")

    # Move the robot to the target position
    movement_from_A_to_landmark(env, robot, start_position, target_position)

    # p.take_pictures()

    # env.Pend()




if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run RCareWorld bathing environment simulation.')
    parser.add_argument('-g', '--graphics', action='store_true', help='Enable graphics')
    parser.add_argument('-d', '--dev', action='store_true', help='Run in developer mode')
    args = parser.parse_args()
    _main(use_graphics=args.graphics, dev=args.dev)
