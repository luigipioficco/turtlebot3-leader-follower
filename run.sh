# Starts the container, with NVIDIA GPU passthrough under WSL.
xhost +
docker run -it --rm --runtime=nvidia --gpus all --net host --ipc host \
    --device /dev/dxg \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/.Xauthority:/root/.Xauthority \
    -v /usr/lib/wsl:/usr/lib/wsl \
    -e DISPLAY=$DISPLAY \
    -e XAUTHORITY=$XAUTHORITY \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e LD_LIBRARY_PATH=/usr/lib/wsl/lib \
    -v ./ros_ws/:/root/ros_workspace \
    --name mr_26_12 \
    mr_26_12:latest bash

# Without a GPU, comment out the block above and use this one instead:
#xhost +
#docker run -it --rm --net host --ipc host \
#    -v /tmp/.X11-unix:/tmp/.X11-unix \
#    -v ~/.Xauthority:/root/.Xauthority \
#    -e DISPLAY=$DISPLAY \
#    -e XAUTHORITY=$XAUTHORITY \
#    -v ./ros_ws/:/root/ros_workspace \
#    --name mr_26_12 \
#    mr_26_12:latest bash
