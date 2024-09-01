#!/bin/bash

echo $RIVA_SERVER_URI

cd /python-clients

# Run get_index.py and extract the device ID for ReSpeaker 4
MIC_IDX=$(python3 get_index.py | grep "ReSpeaker 4" | awk '{print $4}')

# Print the extracted device ID
echo "Found 'ReSpeaker 4' Device ID: $MIC_IDX"

# Create an ASR buffer file and output the ASR results to it
touch /tmp/out.txt && chmod 666 /tmp/out.txt && python3 asr_client.py --server-uri=$RIVA_SERVER_URI --input-device $MIC_IDX >> /tmp/out.txt
