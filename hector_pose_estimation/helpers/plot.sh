#!/bin/bash

rxplot /linear_acceleration_bias/vector/x,/linear_acceleration_bias/vector/y,/linear_acceleration_bias/vector/z /raw_imu/linear_acceleration/x,/raw_imu/linear_acceleration/y,/raw_imu/linear_acceleration/z &
rxplot /angular_velocity_bias/vector/x,/angular_velocity_bias/vector/y,/angular_velocity_bias/vector/z /raw_imu/angular_velocity/x,/raw_imu/angular_velocity/y,/raw_imu/angular_velocity/z &


