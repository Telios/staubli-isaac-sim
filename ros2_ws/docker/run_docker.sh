xhost local:root

XAUTH=/tmp/.docker.xauth

docker run -it \
    --name=ros_noetic \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --net=host \
    --privileged \
    --mount type=bind,src="$(pwd)/src",target=/catkin_ws/src \
    --rm \
    ros_noetic \
    bash
echo "Done. "