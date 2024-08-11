#!/bin/bash

# Launches a Riva server instance.
# Riva clients connect to the server and make requests.

RIVA_SDK_PTH=/mnt/nova_ssd/ngc/riva_quickstart_arm64_v2.16.0

cd $RIVA_SDK_PTH

bash riva_init.sh
bash riva_start.sh

echo "Riva server started"