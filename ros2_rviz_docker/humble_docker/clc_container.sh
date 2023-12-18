sudo docker kill $(sudo docker ps | grep "humble_rviz" | awk '{print $1}')
sudo docker rm $(sudo docker ps -aq)
