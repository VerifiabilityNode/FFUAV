#!/bin/bash

echo "Setting up Catkin workspace"
source devel/setup.bash

for file in src/ros-gen/map/bm_suite/launch/map_*.test;
do
    filename=${file##*/}
    pkgname=${filename/.test}
    echo "Running ROSTest on ${pkgname}"
    mkdir -p results/$pkgname
    /usr/bin/time -o results/$pkgname/rostest.time.txt rostest map_bm_suite ${filename} > results/$pkgname/rostest.out.txt
done
