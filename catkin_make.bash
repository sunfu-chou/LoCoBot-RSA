#!/usr/bin/env bash

catkin_make -C low_cost_ws --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/home/locobot/pyenv_pyrobot_python3/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/home/locobot/pyenv_pyrobot_python3/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so