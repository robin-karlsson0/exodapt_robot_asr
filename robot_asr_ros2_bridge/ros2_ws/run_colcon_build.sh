#!/bin/bash

source /home/$USER/projects/robot_llm/robot_llm_action_server/ros2_ws/install/setup.bash
source /home/$USER/projects/robot_state_manager/ros2_ws/install/setup.bash

# For install dependencies
source /home/$USER/.pyenv/versions/asr_llm_filter/bin/activate
export PYTHONPATH=/home/$USER/.pyenv/versions/asr_llm_filter/lib/python3.10/site-packages:$PYTHONPATH

# Local Robot prompt templates repository
export PYTHONPATH=/home/$USER/projects/robot_prompt_templates/src/:$PYTHONPATH

colcon build --symlink-install
