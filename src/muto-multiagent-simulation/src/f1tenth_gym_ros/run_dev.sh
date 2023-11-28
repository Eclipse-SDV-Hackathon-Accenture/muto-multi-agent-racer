#!/usr/bin/bash
docker run -it --rm --privileged \
  --gpus all \
  --net host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  sim:latest


