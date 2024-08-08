# robot_asr
Robot Automatic speech recognition (ASR) package



### Riva SDK Server

Riva processes runs on a server instance. Clients connect to the server and send requests.

Ref: https://catalog.ngc.nvidia.com/orgs/nvidia/teams/riva/resources/riva_quickstart_arm64

Go to Riva SDK directory
```
$ cd /mnt/nova_ssd/ngc/riva_quickstart_arm64_v2.16.0
```

Set up configuration file
```
$ vim config.sh
    set port
    set languages
    ... ?
```

Start server
```
$ bash riva_init.sh
$ bash riva_start.sh
```

### Riva SDK Client

ROS 2 bridge
```
docker build -t robot_asr .
docker run --runtime nvidia -it --rm --network=host --privileged -v /dev/bus/usb:/dev/bus/usb robot_asr
```

Jetson containers
```
docker run --runtime nvidia -it --rm --network=host --privileged -v /dev/bus/usb:/dev/bus/usb dustynv/riva-client:python-r36.2.0
```

NOTE: Need the --privileged ... for USB devices



