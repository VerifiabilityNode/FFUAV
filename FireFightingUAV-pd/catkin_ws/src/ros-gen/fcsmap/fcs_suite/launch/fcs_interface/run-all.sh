#!/bin/bash
#
# Bash script to run all tests using rostest.
#

for file in fcsmap-*.test;
do
    echo "Running file: $file";
    rostest fcsmap_fcs_suite $file 2>&1 | tee /dev/tty | stdbuf -oL ansi2txt > logs/run_$file.log
done