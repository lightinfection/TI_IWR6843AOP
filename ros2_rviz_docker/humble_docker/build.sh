sudo docker rmi $(sudo docker images | grep "humble_rviz" | awk '{print $3}')
sudo docker build --no-cache -t humble_rviz .
