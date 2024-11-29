from pyrcareworld.envs.bathing_env import BathingEnv
import numpy as np
import cv2
import argparse
import mediapipe as mp

class Perception:

    img_size = 512
    world_x_min = -2
    world_x_max = 2
    world_range_x = world_x_max - world_x_min
    world_z_min = -1
    world_z_max = 3
    world_range_z = world_z_max - world_z_min
    camera_min = 1
    camera_max = img_size
    camera_range = camera_max - camera_min
    camera = None 
    manikin_landmarks = []

    # Initialize Mediapipe Pose
    mp_drawing = mp.solutions.drawing_utils
    mp_pose = mp.solutions.pose

    pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)

    def __init__(self, env:BathingEnv):
        self.environment = env
        self.camera = self.environment.get_camera()
        self.camera.SetTransform(position=[0, 3.8, 1.0], rotation=[90, 0, 0])

    def get_sponge_coords(self) -> list:
        """
        Get the global coordinates of the sponge in the environment
        :param env: the BathingEnv object
        :return: a list of the (x, y, z) coordinates of the sponge object (where y is vertical)
        """
        sponge_details = self.environment.get_sponge().data
        return sponge_details.get("position")

    def get_gripper_coords(self) -> list:
        """
        Get the global coordinates of the sponge in the environment
        :param env: the BathingEnv object
        :return: a list of the (x, y, z) coordinates of the sponge object (where y is vertical)
        """
        gripper = self.environment.get_gripper()
        return gripper.data['position']

    def camera_to_world(self, camera_x, camera_z) -> list:
        """
        Get the global coordinates in the world from the (x, y) pixel coordinate in an image
        :param camera_x: the x-coordinate pixel (horizontal)
        :param camera_z: the y-coordinate pixel (vertical)
        :return: a list of the (x, y, z) world coordinates of the pixel coordinate (where y = 0.0 as height data cannot be inferred)
        """

        world_x = (((camera_x - self.camera_min) * self.world_range_x) / (self.camera_range - 1)) + self.world_x_min
        world_z = ((((self.img_size - camera_z) - self.camera_min) * self.world_range_z) / (self.camera_range - 1)) + self.world_z_min
        return([world_x, 0.0, world_z])

    def find_water_tank(self) -> list:
        """
        Get the global coordinates of the water tank in the environment
        :param env: the BathingEnv object
        :return: a list of the (x, y, z) coordinates of the centre of the water tank object (where y = 0.0 as no vertical data can be inferred)
        """
        self.camera.GetRGB(512, 512)
        self.environment.step()
        rgb = np.frombuffer(self.camera.data["rgb"], dtype=np.uint8)
        rgb = cv2.imdecode(rgb, cv2.IMREAD_COLOR)
        #cv2.imwrite("sky_cam_new.png", rgb)
        
        # Convert to graycsale
        img_gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)

        # Set up the detector with default parameters.
        detector = cv2.SimpleBlobDetector_create()
        # Detect blobs.
        keypoints = detector.detect(img_gray)
        #im_with_keypoints = cv2.drawKeypoints(img_gray, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
        bowl_x = 0
        bowl_z = 0
        size = 0
        for k in keypoints:
            if k.size > size:
                bowl_x = k.pt[0]
                bowl_z = k.pt[1]
                size = k.size

        #DEBUG
        #cv2.circle(im_with_keypoints, (int(x), int(y)), int(size/3), (0,0,255))
        #cv2.circle(im_with_keypoints, (int(camera_coords[0]), int(camera_coords[1])), int(size/3), (0,255,0))
        #cv2.imshow("Keypoints", im_with_keypoints)

        #DEBUG
        #print("bowl location")
        #print(camera_to_world(x, y))

        return self.camera_to_world(bowl_x, 0.0, bowl_z)

    def find_manikin(self, body_part) -> list:
        """
        Get the global coordinates of the manikin's specified body part in the environment
        :param body_part: string name of the body part
        :return: a list of the (x, y, z) coordinates of the body part (where y = 0.0 as no vertical data can be inferred)
        """
        if len(self.manikin_landmarks) == 0:
            self.camera.GetRGB(512, 512)
            self.environment.step()
            rgb = np.frombuffer(self.camera.data["rgb"], dtype=np.uint8)
            rgb = cv2.imdecode(rgb, cv2.IMREAD_COLOR)
            # Process the frame with Mediapipe
            results = self.pose.process(rgb)
            for id, landmark in enumerate(results.pose_landmarks.landmark):
                h, w, _ = rgb.shape  # Get the dimensions of the frame
                cx, cy = int(landmark.x * w), int(landmark.y * h) #convert normalised coordinates to pixels
                self.manikin_landmarks.append([self.mp_pose.PoseLandmark(id).name, cx, cy])
                #DEBUG print(f"ID: {id}, Name: {self.mp_pose.PoseLandmark(id).name}, X: {cx}, Y: {cy}")
                #DEBUG cv2.circle(rgb, (int(cx), int(cy)), 5, (255,0,0), cv2.FILLED)

        #DEBUG cv2.imwrite('skelly.png', rgb)
        for landmark in self.manikin_landmarks:
            if landmark[0] == body_part:
                return self.camera_to_world(landmark[1], landmark[2])

def _main(use_graphics=False):
    # Initialize the environment
    env = BathingEnv(graphics=use_graphics)

    robot = env.get_robot()
    env.step()
    #print(robot.data)

    

    # Control the gripper
    gripper = env.get_gripper()
    #print("gripper info")
    #print(gripper.data)
    gripper.GripperOpen()
    env.step(300)

    gripper.GripperClose()
    env.step(300)

    # Obtain sponge data and simulate a step
    sponge = env.get_sponge()
    #print("sponge location:")
    #print(sponge.data["location"])
    env.step()

    # Camera operations: rig the skycam
    camera = env.get_camera()
    camera.SetTransform(position=[0, 3.8, 1.0], rotation=[90, 0, 0])

    camera.GetRGB(512, 512)
    env.step()
    rgb = np.frombuffer(camera.data["rgb"], dtype=np.uint8)
    rgb = cv2.imdecode(rgb, cv2.IMREAD_COLOR)
    cv2.imshow('original',rgb)
    #cv2.imwrite("sky_cam_new.png", rgb)

    p = Perception(env)
    print(p.find_manikin("RIGHT_ELBOW"))

    

    # Drawing the skeleton 
    # if results.pose_landmarks:
    #     mp_drawing.draw_landmarks(
    #         rgb,
    #         results.pose_landmarks,
    #         mp_pose.POSE_CONNECTIONS,
    #         mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=3),
    #         mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2, circle_radius=3)
    #     )

    #     # joint coordinates
    #     for id, landmark in enumerate(results.pose_landmarks.landmark):
    #         h, w, _ = rgb.shape  # Get the dimensions of the frame
    #         cx, cy = int(landmark.x * w), int(landmark.y * h)  # Convert normalized coordinates to pixel values
    #         print(f"ID: {id}, Name: {mp_pose.PoseLandmark(id).name}, X: {cx}, Y: {cy}")
    #         cv2.circle(rgb, (cx, cy), 5, (255, 0, 0), cv2.FILLED)
        
    #cv2.imshow('Skeleton Detection', rgb)

    #cv2.waitKey(1)

    # for id, landmark in enumerate(results.pose_landmarks.landmark):
    #     h, w, _ = rgb.shape  # Get the dimensions of the frame
    #     cx, cy = int(landmark.x * w), int(landmark.y * h) #convert normalised coordinates to pixels
    #     print(f"ID: {id}, Name: {mp_pose.PoseLandmark(id).name}, X: {cx}, Y: {cy}")

    # Move the robot to some radom pisitions
    # to note, these positions might not be reachable. The robot will try to reach the closest possible position
    position1 = (0.492, 0.644, 0.03)
    position2 = (0.296, 0.849, 3.168)

    robot.IKTargetDoMove(
        position=[position1[0], position1[1] + 0.5, position1[2]],
        duration=2,
        speed_based=False,
    )
    robot.WaitDo()
    robot.IKTargetDoMove(
        position=[position1[0], position1[1], position1[2]],
        duration=2,
        speed_based=False,
    )
    robot.WaitDo()
    robot.IKTargetDoKill()

    # disable IK movement
    # in this mode, you can directly control the robot's joints
    robot.EnabledNativeIK(False)

    gripper.GripperClose()
    env.step(50)
    for i in range(200):
        robot.SetJointVelocity([0, 0, 0, -0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        env.step()

    for i in range(200):
        robot.SetJointVelocity([0, 0, 0, -0.5, -0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        env.step()
    
    for i in range(200):
        robot.SetJointVelocity([0, 0, 0, -0.5, -0.5, -0.5, 0, 0, 0, 0, 0, 0, 0, 0])
        env.step()
    
    
    """
        The Stretch's movement speed is related to the time in the step() method, as well as the defined distance and speed.
        
        - If the `env.step` duration is too short, it can lead to incomplete turns and might cause the robot to move too quickly.

        - If the `env.step` duration is too long, it can lead to slow drifting due to friction.

        - If the speed is too fast, it can cause the robot to move too quickly and fall apart, and it may also result in the robot jumping and falling down.

        - If the speed is too slow, it can lead to the robot not moving at all, only making slight movements, or quickly returning to its original position after moving.

        - If you observe stretching and contracting in the robot's arm, this is due to angular momentum. Reducing speed can lessen this effect. Additionally, we recommend lowering the robot's arm during movement to lower the center of gravity, effectively reducing this issue and ensuring arm stability.

        Below is a simple example where the robot can move smoothly using these parameters, though there is significant room for adjustment.

        Particularly, we do not recommend continuous motion as it can lead to great instability. It is better to interrupt and halt movement intermittently to reduce continuous motion.
    """
    robot.TurnLeft(90, 1)
    env.step(400)

    env.step(200)
    robot.StopMovement()

    
    robot.TurnRight(90, 1)
    env.step(600)
    
    # robot.StopMovement()
    # env.step(30)
    
    robot.MoveForward(0.6, 0.2)
    env.step(300)


    # Additional simulation logic can be added here
    # For example:
    print("Force", sponge.GetForce())
    env.step()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run RCareWorld bathing environment simulation.')
    parser.add_argument('-g', '--graphics', action='store_true', help='Enable graphics')
    args = parser.parse_args()
    _main(use_graphics=args.graphics)
