#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import matplotlib.pyplot as plt
import matplotlib.patches as pat
import numpy as np
import math as m

# Leg kinematics design parameters
h_min       = 50.0
h_max       = 220.0
l           = 125.0
vdr         = 80.0
rk          = 15.0
rd_init     = 4.0
f           = 30.0
# Calculation parameter
hh_init     = 2.5
# Servo motor specification & leg parameters
motor_stall_torque_Nm       = 8.9
motor_noload_rpm            = 66
max_apply_force_N           = motor_stall_torque_Nm / vdr / 10E3
max_apply_force_kgf         = max_apply_force_N / 9.81
max_extention_speed_mms     = motor_noload_rpm * vdr * m.pi / 30
max_extention_time_ms       = 250 / max_extention_speed_mms * 10E3
driver_angle_range_deg      = (h_max - h_min) / vdr
# Const
ddar        = hh_init / vdr
num_loop    = int((h_max - h_min) / hh_init) + 1
# Visual
visual_value        = 1
visual_wire         = 0
visual_controller   = 1


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


def getRotP(_p, _rot):
    rot_v   = np.array( [[np.cos(_rot), -np.sin(_rot)], [np.sin(_rot),  np.cos(_rot)]] )
    return np.dot( rot_v, _p )


def get_psi(_h):
    return np.arcsin(_h / (2*l))


def get_gar(_rd):
    return np.arcsin( (rk - _rd) / f )


def get_P(_rd):
    return _rd * np.sin( get_gar(_rd) ), _rd * np.cos( get_gar(_rd) )

def get_t(_rd):
    t_sq = m.pow(f, 2) - m.pow(rk - _rd, 2)
    if t_sq >= 0:
        return m.sqrt( t_sq )
    else:
        print('error t_sq < 0')

def get_y(_rd):
    return rk * get_gar(_rd) + get_t(_rd)


def searchCurrentRd(_rd, _ddar, _d_xt, gain = 0.2, pass_error = 10E-5):
    # Val for visual
    global cnt, vis_t, vis_ref, vis_cur, vis_rd
    # Var for feedback calculation
    y0 = get_y(_rd)
    P0 = getRotP(get_P(_rd), _ddar * -1)
    P  = get_P(_rd)
    while 1 :
        u   = np.linalg.norm(P - P0)
        e   = (y0 + _d_xt) - (get_y(_rd) + u)
        _rd += e * gain
        P = get_P(_rd)
        if visual_controller:
            vis_t.append(cnt)
            vis_ref.append(y0 + _d_xt)
            vis_cur.append(get_y(_rd) + u)
            vis_rd.append(_rd)
            cnt += 1
        if m.fabs(e) < pass_error:
            break
    return _rd

if __name__ == '__main__':
    # Leg entire structure
    h    = np.array( [h_min], dtype=float )     # Distance between upper pitch axis and lower pitch axis [mm]
    dar  = np.array( [0], dtype=float )                     # Angle of driver [rad]
    psi  = np.array( [get_psi(h[0])], dtype=float )         # Knee inside angle [rad]
    d_xt = np.array( [0], dtype=float )                     # Wire extension length diff
    # Driver pulley
    rd   = np.array( [rd_init], dtype=float ) # Driver radius
    gar  = np.array( [get_gar(rd)], dtype=float)            # Wire gradient
    P    = np.array( [get_P(rd)], dtype=float)              # Driver pulley action point
    # Var for visual
    cnt     = 0
    vis_t   = []
    vis_ref = []
    vis_cur = []
    vis_rd  = []
    
    for n in range(1, num_loop):
        h    = np.append( h, h[n-1] + hh_init )
        dar  = np.append( dar, dar[n-1] + ddar )
        psi  = np.append( psi, get_psi(h[n]) )
        d_xt = np.append( d_xt, rk * (psi[n] - psi[n-1]) )
        # Driver pulley
        rd   = np.append( rd, searchCurrentRd(rd[n-1], ddar, d_xt[n]) )
        gar  = np.append( gar, get_gar(rd[n]) )
        P    = np.append( P, [get_P(rd[n])] )
        # Visual wire

    if visual_value:
        print('dr range:%.1f' % m.degrees(driver_angle_range_deg))
        print('h\tdar\tpsi\td_xt\tgar\trd')
        for n in range(0, num_loop):
            print('%.1f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t' % (h[n], dar[n], psi[n], d_xt[n], gar[n], rd[n]))

    if visual_controller:
        plt.figure(figsize=(6,12))

        plt.subplot(311)
        plt.plot(vis_t, vis_ref, label='ref')
        plt.plot(vis_t, vis_cur, label='cur')
        plt.title('Feedback')
        plt.legend()

        plt.subplot(312)
        plt.plot(vis_t, vis_rd, label='rd')
        plt.title('Rd')
        plt.legend()

        # Knee vector visual init
        ax_wire  = plt.subplot(313)
        plt.title('Knee coordinate')
        coordinate(ax_wire, [-20, 80], [-25, 25])

        for n in range(0, num_loop):
            wedge = pat.Wedge(center=(0, 0), r=rk, theta1=90 - m.degrees(gar[n]), theta2=90, fc="white", ec="red")
            ax_wire.add_patch( wedge )
            ax_wire.plot( [rk * np.sin(gar[n]),f + rd[n] * np.sin(gar[n])], [rk * np.cos(gar[n]), rd[n] * np.cos(gar[n])] )
            ax_wire.plot( [f, f + rd[n] * np.sin(gar[n])], [0, rd[n] * np.cos(gar[n])] )

        plt.figure(figsize=(6,6))
        ax2 = plt.subplot(111)
        coordinate(ax2, [-15, 15], [-15, 15])
        plt.title('Driver pulley')
        plt.plot([0, rd[0] * np.sin(gar[0] - dar[0])], [0, rd[0] * np.cos(gar[0] - dar[0])], color='black')
        for n in range(1, num_loop):
            plt.plot([0, rd[n] * np.sin(gar[n] - dar[n])], [0, rd[n] * np.cos(gar[n] - dar[n])], color='blue')
            dif_x = [rd[n-1] * np.sin(gar[n-1] - dar[n-1]), rd[n] * np.sin(gar[n] - dar[n])]
            dif_y = [rd[n-1] * np.cos(gar[n-1] - dar[n-1]), rd[n] * np.cos(gar[n] - dar[n])]
            plt.plot(dif_x, dif_y, color='black')
        plt.plot([rd[n] * np.sin(gar[n] - dar[n]), 0], [rd[n] * np.cos(gar[n] - dar[n]),0], color='black')

        plt.show()

    # if visual_wire:
        
        # plt.show()
