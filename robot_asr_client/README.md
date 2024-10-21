# robot_asr
Robot Automatic speech recognition (ASR) package



# Riva SDK Server

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

# Riva SDK Client

### ASR Client

Creates and reads a file `/tmp/tts_is_speaking.txt` presumed to contain `0` or `1` indicating if TTS is running.

The  `_fill_buffer()` callback function in `MicrophoneStream` only buffer audio data from mic when `0`.

NOTE: Presumes the TTS node updates the state!

### ROS 2 bridge

First, run the containerized ASR client that transmits microphone data to the Riva server and appends the resulting text to a file. The file is readable and writable by users outside the container, and is used as a text buffer.

The ROS bridge node reads the text buffer file, publish the content to a ROS topic, and empties the file.

NOTE: Make sure the `/tmp/out.txt` file does not exist before running the ASR client!

```
$ docker build -t robot_asr .
$ sh riva_client_start.sh

# Confirm microphone `Input Device id`
$ python3 get_index.py

# Run ASR client that outputs speech to text file with full permissions
$ touch /tmp/out.txt && chmod 666 /tmp/out.txt && python3 asr_client.py --server-uri=172.20.137.207:50051 --input-device 4 >> /tmp/out.txt

# Run ROS bridge node that reads outputted text and publishes to topic

```

### Jetson containers
```
docker run --runtime nvidia -it --rm --network=host --privileged -v /dev/bus/usb:/dev/bus/usb dustynv/riva-client:python-r36.2.0
```

NOTE: Need the --privileged ... for USB devices



