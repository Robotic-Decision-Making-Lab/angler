# Life of a Pose

1. Get the pose of the marker in the `camera` frame
2. Transform the pose from the `camera` frame to the `marker` frame (parent: `camera`, child: `marker`)
3. Transform the pose from the `marker` frame to the `map` frame (parent: `marker`, child: `map`)
4. Transform the pose from the `map` frame to the ArduSub `world` frame (parent: `map`, child: `world`)
5. Publish the pose to the ArduSub EKF
6. The ArduSub EKF fuses all sensor information to create a new pose in the `world` frame
7. The pose is converted from the `world` frame to the `map` frame (parent: `world`, child: `map`)
