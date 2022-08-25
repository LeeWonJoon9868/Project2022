#!/bin/bash

rosbag record --split --size 1024 -o ml_training /camera_rgb /cmd_vel /ackermann_cmd

