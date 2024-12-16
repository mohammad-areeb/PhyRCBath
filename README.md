# PhyRCBath
RoboNotts development for the PhyRC Challenge

## Perception
The Perception class is contained in `bathing_perception.py`. To use this code you will need to create an instance of the Perception class and pass in the `BathingEnv` variable. There is a minimal working version of how to use the Perception class in `bathing_perception_main.py`.

### Implementation notes for Perception 
- RCareWorld has the y-axis pointing up. If the y-axis value cannot be determined from a 2D image, the Perception class will return the y-axis value as zero.
- Mediapipe landmarks can be found [here.](https://ai.google.dev/edge/mediapipe/solutions/vision/pose_landmarker) The manikin's landmarks can either be referenced using the landmark number, or the landmark name (all uppercase, spaces replaced with underscores). 
