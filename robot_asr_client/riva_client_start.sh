#!/bin/bash

# IP Addresses
export RIVA_SERVER_URI=172.20.137.162:50051

docker run -it --rm \
    --network=host \
    --privileged \
    -v /dev/bus/usb:/dev/bus/usb \
    -v /tmp:/tmp \
    -e RIVA_SERVER_URI \
    robot_asr \