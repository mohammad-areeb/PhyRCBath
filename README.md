# PhyRCBath
RoboNotts development for the PhyRC Challenge.

Run `robonotts_bathing_main.py` for current working solution. 

## Perception
The Perception class is contained in `bathing_perception.py`. To use this code you will need to create an instance of the Perception class and pass in the `BathingEnv` variable. There is a minimal working version of how to use the Perception class in `bathing_perception_test.py`.

### Implementation notes for Perception 
- You must call generate_landmarks() before you can access the landmarks of the manikin. This is a change from the previous version of the code. 
- RCareWorld has the y-axis pointing up. If the y-axis value cannot be determined from a 2D image, the Perception class will return the y-axis value as zero.
- Depth is returned as the landmark's height from the floor, not the distance from the camera.
- Mediapipe landmarks can be found [here.](https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker) The manikin's landmarks can either be referenced using the landmark number, or the landmark name (all uppercase, spaces replaced with underscores). 
