#!/bin/bash
set -e

to=$1
name=$2
image=$3
ws_dir=$4
net=$5
run_cmd=$6
external_port=$7

# run the deteched docker container
cont=$(docker run -d -p $external_port:9090 \
  -v "$ws_dir":/home/root/catkin_ws/src -w /home/root/catkin_ws/ \
  --name "$name" "$image" \
  /bin/bash -c "stdbuf -o L $run_cmd")
# stream stdout outside the containter
docker logs --follow "$name" &
# timeout the docker containter if it runs too long
code=$(timeout "$to" docker wait "$cont" || true)
# kill the docker containter
docker rm --force $cont &> /dev/null
