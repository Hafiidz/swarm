### Recommended Steps

# ROS drone show swarm project

## Project setup & resources
1. ros : this is the repo for Dockerfile and coordinate generation code https://github.com/Hafiidz/ros
1. swarm : this is the  repo for the swarm via turtlesim source code https://github.com/Hafiidz/swarm

## Steps
1. Install Docker Desktop/Docler https://docs.docker.com/get-docker/
1. Run the following in the host terminal `docker run -p 6080:80 --shm-size=512m --name=rostest -it hafiidz/noetic-swarm:v0`
1. Open http://localhost:6080/ on your host PC browser
1. Inside the container, open a terminal and run: `roslaunch swarm central.launch`


