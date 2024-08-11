# robot_asr
Robot Automatic speech recognition (ASR) package

# Riva SDK Server

Riva processes runs on a server instance. Clients connect to the server and send requests.

### Configure Riva SDK Server

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

### Start Riva SDK Server

Start Riva SDK server by running the startup script
```
sh riva_server_startup.sh
```

Ref: https://catalog.ngc.nvidia.com/orgs/nvidia/teams/riva/resources/riva_quickstart_arm64
