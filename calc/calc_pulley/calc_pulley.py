#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import matplotlib.pyplot as plt
import numpy as np
import math as m

# Leg kinematics design parameters
min_leg_height_mm           = 50.0
max_leg_height_mm           = 100.0
leg_length                  = 125
virtual_driver_radius_mm    = 95
knee_pulley_radius_mm       = 14
driver_init_radius_mm       = 5
knee_driver_distance_mm     = 60
# Calculation parameter
leg_height_step_mm          = 5.0

# Servo motor specification parameters
motor_stall_torque_Nm       = 8.9
motor_noload_rpm            = 66

# Fix calculation based parameters
max_apply_force_N           = motor_stall_torque_Nm / virtual_driver_radius_mm / 10E3
max_apply_force_kgf         = max_apply_force_N / 9.81
max_extention_speed_mms     = motor_noload_rpm * virtual_driver_radius_mm * m.pi / 30
max_extention_time_ms       = 250 / max_extention_speed_mms * 10E3
driver_angle_range_deg      = (max_leg_height_mm - min_leg_height_mm) / virtual_driver_radius_mm

def showGraph():
    plt.plot( [3,1,4,1,5,9,2,6,5] )
    plt.show()
 
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
    # Val
    r_d = driver_init_radius_mm
    # Const
    r_k = knee_pulley_radius_mm
    l   = leg_length
    f   = knee_driver_distance_mm
    # Array
    h   = np.array( [min_leg_height_mm], dtype=float )
    psi = np.array( [np.arcsin(h / (2*l))], dtype=float )
    g   = np.array( [np.arcsin( (r_k - r_d) / f )], dtype=float)
    s   = np.array( [r_k * g], dtype=float )
    t   = np.array( [getSquareFromTwoValue( f, (r_d - r_k) )], dtype=float )
    u   = np.array( [0], dtype=float )
    W   = np.array( [s + t + u], dtype=float )
    X   = np.array( [0], dtype=float )

    for i in range(1, int((max_leg_height_mm - min_leg_height_mm) / leg_height_step_mm)):
        h   = np.append( h  , h[i-1] + leg_height_step_mm )
        psi = np.append( psi, np.arcsin(h[i] / (2*l)) )
        X   = np.append( X  , [r_k * (psi[i]-psi[i-1])] )
        # g   = np.append( g  , np. np.arcsin( (r_k - r_d) / f ))

    np.set_printoptions(precision=1, floatmode='fixed')
    print(h)
    np.set_printoptions(precision=3, floatmode='fixed')
    print(psi)
    print(X)
    # showGraph()
