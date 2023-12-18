xhost +local:
sudo docker run -it --rm \
					--privileged \
					--gpus all \
					--network host \
					--device=/dev/dri \
					--group-add video \
					--volume=/tmp/.X11-unix:/tmp/.X11-unix \
					--env="DISPLAY=$DISPLAY" \
					humble_rviz bash
