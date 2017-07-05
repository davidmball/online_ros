#!/bin/bash
set -e

to=$1
name=$2
image=$3
ws_dir=$4
net=$5
run_cmd=$6
shift
shift
shift
shift
shift
shift

# Create network - disabled for the moment
#docker network create --subnet=172.18.0.0/16 $net &> /dev/null
# run the deteched docker container
#cont=$(docker run --rm -d -p 172.17.0.22:9090:9090 -v "$ws_dir":/home/root/catkin_ws/src -w /home/root/catkin_ws/ --name "$name" "$image" /bin/sh -c "roslaunch rosbridge_server rosbridge_websocket.launch & catkin_make && stdbuf -o L rosrun topics chatter")
cont=$(docker run --rm -d -p 9090:9090 -v "$ws_dir":/home/root/catkin_ws/src -w /home/root/catkin_ws/ --name "$name" "$image" /bin/sh -c "roslaunch rosbridge_server rosbridge_websocket.launch & catkin_make && stdbuf -o L $run_cmd")
# stream stdout outside hte containter
docker logs --follow "$name" &
# timeout the docker containter if it runs too long
code=$(timeout "$to" docker wait "$cont" || true)
# kill the docker containter
docker kill $cont &> /dev/null
# remove the docker network
#docker network rm $net &> /dev/null
