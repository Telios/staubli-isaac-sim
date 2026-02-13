xhost local:root # has to be called once for allowing GUI for docker

docker exec -it docker-ros1_camera-1 bash -c "source /opt/ros/noetic/setup.bash && rosrun rqt_image_view rqt_image_view topic:=/ros1_output_image"