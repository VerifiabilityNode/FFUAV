#!/bin/bash

for file in m1_fcsmap-*.test;
do
    echo "Running file: $file";
    rostest fcsmap_fcs_suite $file 2>&1 | tee /dev/tty | stdbuf -oL ansi2txt > logs/run_$file.log
done