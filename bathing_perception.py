from pyrcareworld.envs.bathing_env import BathingEnv
import numpy as np
import cv2
import mediapipe as mp

class Perception:

    #class variables
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
        
        # Convert to graycsale
        img_gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
        # Set up the detector with default parameters.
        detector = cv2.SimpleBlobDetector_create()
        # Detect blobs.
        keypoints = detector.detect(img_gray)
        
        bowl_x = 0
        bowl_z = 0
        size = 0
        for k in keypoints:
            if k.size > size:
                bowl_x = k.pt[0]
                bowl_z = k.pt[1]
                size = k.size

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