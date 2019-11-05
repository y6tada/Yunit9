#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import matplotlib.pyplot as plt
import matplotlib.patches as pat
import numpy as np
import math as m

# Leg kinematics design parameters
min_leg_height_mm           = 50.0
max_leg_height_mm           = 100.0
leg_length                  = 125
virtual_driver_radius_mm    = 95
knee_pulley_radius_mm       = 14
driver_init_radius_mm       = 5.0
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
# Const
r_k = knee_pulley_radius_mm
vdr = virtual_driver_radius_mm
l   = leg_length
f   = knee_driver_distance_mm
dd  = leg_height_step_mm / virtual_driver_radius_mm * -1


def coordinate(axes, range_x, range_y, grid = True, xyline = True, xlabel = "x", ylabel = "y"):
    axes.set_xlabel(xlabel, fontsize = 12)
    axes.set_ylabel(ylabel, fontsize = 12)
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


def getSq(value1, value2):
    return m.sqrt( m.pow(value1, 2) + m.pow(value2, 2) )


def getRotP_dd(_p):
    rot_v   = np.array( [[np.cos(dd), -np.sin(dd)], [np.sin(dd),  np.cos(dd)]] )
    return np.dot( rot_v, _p )


def get_psi(_h):
    return np.arcsin(_h / (2*l))


def get_g(_rd):
    return np.arcsin( (r_k - _rd) / f )


def get_P(_rd):
    return _rd * np.sin( get_g(_rd) ), _rd * np.cos( get_g(_rd) )


def get_W(_rd):
    return r_k * get_g(_rd) + getSq( f, (_rd - r_k) )


def get_ctrl_rd(prev_rd, ref, gain, error=10E-6):
    # Previous p を1ステップ回転させる
    prev_p = [prev_rd * np.sin( get_g(prev_rd) ), prev_rd * np.cos( get_g(prev_rd) )]
    prev_p = getRotP_dd(prev_p)
    # フィードバック
    crtl_rd = prev_rd
    while 1 :
        cur_w = get_W(crtl_rd)
        cur_p = [ crtl_rd * np.sin(get_g(crtl_rd)), crtl_rd * np.cos(get_g(crtl_rd)) ]
        cur_u = np.linalg.norm(cur_p - prev_p)
        cur_error = ref - (cur_w + cur_u)
        crtl_rd += cur_error * gain
        print('y:%.3f\tref:%.3f\te:%.3f\tcur_u:%.3f' % (cur_w + cur_u, ref, cur_error, cur_u))
        # print('ref:%.3f' % ref)
        # print('cur_u:%.3f' % cur_u)

        if m.fabs(cur_error) < error :
            print('braek')
            break

    return crtl_rd


def visual():
    # fig, ax = plt.subplots(1, 1, figsize=(5, 5))

    # zero    = np.array([0, 0])
    # P_rot   = getRotP_dd(P)

    # plotVector(ax, P, P_rot - P, 6, color="green")
    # plotVector(ax, zero, P, 6, color="red")
    # plotVector(ax, zero, P_rot, 6, color="blue")

    # np.set_printoptions(precision=2, floatmode='fixed')
    # print('h\t',h)
    # print('psi\t',psi)
    # print('X\t',X)
    # print('W\t',W)
    # print('d\t',d)
    # print('\np\n',p)
    
    plt.show()


if __name__ == '__main__':
    # Val
    r_d = driver_init_radius_mm
    # Array
    h   = np.array( [min_leg_height_mm], dtype=float )      # Distance between upper pitch axis and lower pitch axis [mm]
    d   = np.array( [0], dtype=float )                      # Angle of driver [rad]
    psi = np.array( [get_psi(h[0])], dtype=float )          # Knee inside angle [rad]
    
    g   = np.array( [get_g(r_d)], dtype=float)
    w   = np.array( [get_W(r_d)], dtype=float )             # wire length through knee pulley and air              
    P   = np.array( [get_P(r_d)], dtype=float)              # Driver pulley plot point

    for n in range(1, int((max_leg_height_mm - min_leg_height_mm) / leg_height_step_mm)):
        h   = np.append( h  , h[n-1] + leg_height_step_mm )
        psi = np.append( psi, np.arcsin(h[n] / (2*l)) )
        d   = np.append( d  , [(h[n] - min_leg_height_mm) / vdr] )
        
        xx  = r_k * (psi[n] - psi[n-1])
        print('xx:%.3f' % xx)
        wx  = w[n-1] + r_k * (psi[n] - psi[n-1])
        r_d = get_ctrl_rd(r_d, wx, -1.0)
        w   = np.append( w  , [get_W(r_d)] )
        P   = np.append( P  , [get_P(r_d)] )

    # Visual
    fig = plt.figure(figsize=(10,5))
    plt.title('Knee coordinate')
    ax = plt.subplot(111)
    coordinate(ax, [-20, 80], [-25, 25])

    knee_pulley_wire = pat.Wedge(center = (0, 0), r = r_k, theta1 = 90 - m.degrees(g), theta2 = 90, fc = "white", ec="red")
    ax.add_patch(knee_pulley_wire)
    ax.plot( [r_k * m.sin(g),f + r_d * m.sin(g)], [r_k * m.cos(g), r_d * m.cos(g)] )
    ax.plot( [f, f + r_d * m.sin(g)], [0, r_d * m.cos(g)] )

    plt.show()
