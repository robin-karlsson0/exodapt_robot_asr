FROM ubuntu:22.04

RUN apt-get -y update && apt-get upgrade -y && apt-get install -y \
    git \
    vim \
    python3-pip \
    portaudio19-dev \
    && rm -rf /var/lib/apt/lists/*

# Riva Python client
RUN git clone https://github.com/nvidia-riva/python-clients.git
WORKDIR /python-clients
RUN git submodule init
RUN git submodule update --remote --recursive
RUN pip install -r requirements.txt
RUN python3 setup.py bdist_wheel
RUN pip install --force-reinstall dist/*.whl

# Additional Riva Python client dependencies
RUN pip install PyAudio

# RUN adduser $USER audio
RUN adduser $USER pulse-access

COPY asr_client.py .
COPY get_index.py .

CMD ["/bin/bash"]