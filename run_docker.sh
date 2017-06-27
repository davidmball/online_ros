#!/bin/bash
set -e

to=$1
name=$2
image=$3
ws_dir=$4
shift
shift
shift
shift

#cont=$(docker run --rm -d --name "$name" "$image" "$@" /bin/sh -c "while true; do echo hello; sleep 1; done;")
docker network create --subnet=172.18.0.0/16 mynet123 &> /dev/null
cont=$(docker run --rm -d --net mynet123 --ip 172.18.0.22 -p 9090 -v "$ws_dir":/home/root/catkin_ws/src -w /home/root/catkin_ws/ --name "$name" "$image" /bin/sh -c "roslaunch rosbridge_server rosbridge_websocket.launch & catkin_make && stdbuf -o L rosrun topics chatter")
docker logs --follow "$name" &
code=$(timeout "$to" docker wait "$cont" || true)
docker kill $cont &> /dev/null
docker network rm mynet123  &> /dev/null
#echo -n 'status: '
#if [ -z "$code" ]; then
#    echo timeout
#else
#    echo exited: $code
#fi
