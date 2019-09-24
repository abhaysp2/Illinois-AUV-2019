#! /bin/bash

set -x

path_to_dir="~/ros/auv/"

source $path_to_dir/devel/setup.bash

roslaunch robosub_2019 start_simulation.launch &

sleep 5

roslaunch robosub_2019 start_thruster.launch &

trap 'kill $(jobs -p); killall gzserver; killall gzclient' SIGINT SIGTERM EXIT

wait
