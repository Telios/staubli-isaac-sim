xhost local:root

XAUTH=/tmp/.docker.xauth

docker run -it \
    --name=detectnet \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --net=host \
    --ipc=host \
    --pid=host \
    --privileged \
    --runtime nvidia \
    --rm \
    docker-ros2_detectnet \
    bash
echo "Done. "