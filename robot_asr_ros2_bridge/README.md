# Robot ASR
Robot Automatic speech recognition (ASR) package

# ROS 2 Riva SDK Bridge

The ROS bridge node reads the text buffer file, publish the content to a ROS topic, and empties the file.

Build and install ROS 2 package
```
$ cd ros2_ws
$ colcon build
$ source install/setup.bash
```

Run ROS 2 package
```
ros2 run asr_riva_bridge asr_riva_bridge
```

### NOTE: Do NOT use Docker container

Build docker container
```
$ docker build -t robot_asr_bridge .
```

Run docker container 
```
docker run -it --rm --network=host -v /tmp:/tmp robot_asr_bridge
```
