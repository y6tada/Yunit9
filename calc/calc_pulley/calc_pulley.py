#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
# from mpl_toolkits.mplot3d import Axes3D
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

def coordinate(axes, range_x, range_y, grid = True, xyline = True, xlabel = "x", ylabel = "y"):
    axes.set_xlabel(xlabel, fontsize = 16)
    axes.set_ylabel(ylabel, fontsize = 16)
    axes.set_xlim(range_x[0], range_x[1])
    axes.set_ylim(range_y[0], range_y[1])
    if grid == True:
        axes.grid()
    if xyline == True:
        axes.axhline(0, color = "gray")
        axes.axvline(0, color = "gray")

def plotVector(axes, loc, vector, lim, color = "red"):
    axes.quiver(loc[0], loc[1], vector[0], vector[1], color = color, angles = 'xy', scale_units = 'xy', scale = 1)

def dataPlot():
    file = "val_trace.txt"
    with open(file, "w", encoding = "utf_8") as fileobj:
        args = sys.argv
        fileobj.write(args[0])

def getDistanceFromTwoCoordinates(p1, p2):
    return getSq( (p1[0] - p2[0]), (p1[1] - p2[1]) )

def getSq(value1, value2):
    return m.sqrt( m.pow(value1, 2) + m.pow(value2, 2) )

def getRotP(p, d):
    dd      = (d[i] - d[i-1]) * -1
    # p_vert  = p.reshape(1, 2)
    rot_v   = np.array( [[np.cos(dd), -np.sin(dd)], [np.sin(dd),  np.cos(dd)]] )
    return np.dot( rot_v, p )

if __name__ == '__main__':
    # Const
    r_k = knee_pulley_radius_mm
    vdr = virtual_driver_radius_mm
    l   = leg_length
    f   = knee_driver_distance_mm
    # Val
    r_d = driver_init_radius_mm
    # Array
    h   = np.array( [min_leg_height_mm], dtype=float )
    d   = np.array( [0], dtype=float )
    psi = np.array( [np.arcsin(h / (2*l))], dtype=float )
    g   = np.array( [np.arcsin( (r_k - r_d) / f )], dtype=float)
    p   = np.array( [float(r_d * np.sin(g)), float(r_d * np.cos(g))], dtype=float)

    W   = np.array( [r_k * g + getSq( f, (r_d - r_k) )], dtype=float )
    X   = np.array( [0], dtype=float )

    for i in range(1, int((max_leg_height_mm - min_leg_height_mm) / leg_height_step_mm)):
        h   = np.append( h  , h[i-1] + leg_height_step_mm )
        psi = np.append( psi, np.arcsin(h[i] / (2*l)) )
        X   = np.append( X  , [r_k * (psi[i]-psi[i-1])] )
        d   = np.append( d  , [(h[i] - min_leg_height_mm) / vdr] )

        # g   = np.append( g  , np. np.arcsin( (r_k - r_d) / f ))

    fig, ax = plt.subplots(1, 1, figsize=(5, 5))
    coordinate(ax, [-6, 6], [-6, 6])
    zero    = np.array([0, 0])
    p_rot   = getRotP(p, d)

    plotVector(ax, zero, p, 6, color="red")
    plotVector(ax, zero, p_rot, 6, color="blue")
    plt.show()

    np.set_printoptions(precision=2, floatmode='fixed')
    # print('h\t',h)
    # print('psi\t',psi)
    # print('X\t',X)
    # print('W\t',W)
    # print('d\t',d)
    # print('\np\n',p)
    # showGraph()
