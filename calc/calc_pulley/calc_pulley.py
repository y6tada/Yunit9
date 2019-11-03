#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import matplotlib.pyplot as plt
import numpy as np
import math as m

# Leg kinematics design parameters
min_leg_height_mm           = 50.0
max_leg_height_mm           = 250.0
leg_length                  = 125
virtual_driver_radius_mm    = 95
knee_pulley_radius_mm       = 14
driver_init_radius_mm       = 5
knee_driver_distance_mm     = 60
# Calculation parameter
leg_height_step_mm          = 0.5

# Servo motor specification parameters
motor_stall_torque_Nm       = 8.9
motor_noload_rpm            = 66

# Fix calculation based parameters
max_apply_force_N           = motor_stall_torque_Nm / virtual_driver_radius_mm / 10E3
max_apply_force_kgf         = max_apply_force_N / 9.81
max_extention_speed_mms     = motor_noload_rpm * virtual_driver_radius_mm * m.pi / 30
max_extention_time_ms       = 250 / max_extention_speed_mms * 10E3
driver_angle_range_deg      = (max_leg_height_mm - min_leg_height_mm) / virtual_driver_radius_mm

def dataPlot():
    file = "val_trace.txt"
    with open(file, "w", encoding = "utf_8") as fileobj:
        args = sys.argv
        fileobj.write(args[0])

def getDistanceFromTwoCoordinates(p1, p2):
    return getSquareFromTwoValue( (p1[0] - p2[0]), (p1[1] - p2[1]) )

def getSquareFromTwoValue(value1, value2):
    return m.sqrt( m.pow(value1, 2) + m.pow(value2, 2) )

if __name__ == '__main__':
    # Const
    r_k = knee_pulley_radius_mm
    r_d = driver_init_radius_mm
    f   = knee_driver_distance_mm
    # Array
    h = np.array([min_leg_height_mm], dtype=float)
    g   = m.asin( (r_k - r_d) / f )
    s   = r_k * g
    t   = getSquareFromTwoValue( f, (r_d - r_k) )
    u   = 0
    W   = s + t + u

    for i in range(0, int((max_leg_height_mm - min_leg_height_mm) / leg_height_step_mm)):
        h = np.append(h, h[i] + leg_height_step_mm)

    print(h)