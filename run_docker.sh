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

# run the deteched docker container
cont=$(docker run -d -p 9090:9090 \
  -v "$ws_dir":/home/root/catkin_ws/src -w /home/root/catkin_ws/ \
  --name "$name" "$image" \
  /bin/sh -c "roslaunch rosbridge_server rosbridge_websocket.launch \
  & rosrun tf2_web_republisher tf2_web_republisher \
  & catkin_make \
  && stdbuf -o L $run_cmd")
# stream stdout outside the containter
docker logs --follow "$name" &
# timeout the docker containter if it runs too long
code=$(timeout "$to" docker wait "$cont" || true)
# kill the docker containter
docker kill $cont &> /dev/null
