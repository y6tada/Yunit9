#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import matplotlib.pyplot as plt
import numpy as np
import math as m

# Leg kinematics design parameters
min_leg_height_mm       	= 50
max_leg_height_mm       	= 250
leg_length              	= 125
virtual_driver_radius_mm	= 95
knee_pulley_radius_mm   	= 14
driver_init_radius_mm   	= 5
knee_driver_distance_mm 	= 60

# Servo motor specification parameters
motor_stall_torque_Nm   	= 8.9
motor_noload_rpm        	= 66

# Fix calculation based parameters
max_apply_force_N       	= motor_stall_torque_Nm / (virtual_driver_radius_mm / 10E3)
max_apply_force_kgf     	= max_apply_force_N / 9.81
max_extention_speed_mms 	= motor_noload_rpm * (virtual_driver_radius_mm * m.pi) / 30
max_extention_time_ms      	= 250 / (max_extention_speed_mms) * 10E3
driver_angle_range      	= (max_leg_height_mm - min_leg_height_mm) / virtual_driver_radius_mm

def data_plot():
	file = "val_trace.txt"
	with open(file, "w", encoding = "utf_8") as fileobj:
	    args = sys.argv
	    fileobj.write(args[0])

data_plot()