# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.
#
# Purpose of this file : Kalman filter class
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

# imports
import numpy as np

# add project directory to python path to enable relative imports
import os
import sys
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))
import misc.params as params

class Filter:
    '''Kalman filter class'''
    def __init__(self):
        self.dt = params.dt
        self.q = params.q
        self.dim_state = params.dim_state

    def F(self):
        ############
        # TODO Step 1: implement and return system matrix F
        ############

        F = np.eye((self.dim_state))
        F = np.asmatrix(F)
        F[0, 3] = self.dt
        F[1, 4] = self.dt
        F[2, 5] = self.dt
        return F
        
        ############
        # END student code
        ############ 

    def Q(self):
        ############
        # TODO Step 1: implement and return process noise covariance Q
        ############
        dt2 = self.dt ** 2
        dt3 = self.dt ** 3
        q_l1 = dt3 * self.q / 3.0
        q_l3 = dt2 * self.q / 2.0
        q_33 = self.dt * self.q
        return np.matrix([[q_l1, 0, 0, q_l3, 0, 0],
                            [0, q_l1, 0, 0, q_l3, 0],
                            [0, 0, q_l1, 0, 0, q_l3],
                            [q_l3, 0, 0, q_33, 0, 0],
                            [0, q_l3, 0, 0, q_33, 0],
                            [0, 0, q_l3, 0, 0, q_33]])
        
        ############
        # END student code
        ############ 

    def predict(self, track):
        ############
        # TODO Step 1: predict state x and estimation error covariance P to next timestep, save x and P in track
        ############

        x_ = self.F() * track.x
        P_ = self.F() * track.P * self.F().transpose() + self.Q()

        track.set_x(x_)
        track.set_P(P_)
        
        ############
        # END student code
        ############ 

    def update(self, track, meas):
        ############
        # TODO Step 1: update state x and covariance P with associated measurement, save x and P in track
        ############
        H = meas.sensor.get_H(track.x)
        gamma = self.gamma(track, meas)
        S = self.S(track, meas, H)
        I = np.asmatrix(np.eye((self.dim_state)))
        K = track.P * H.T * S.I 
        x = track.x + K * gamma
        P = (I - K * H) * track.P
        track.set_P(P)
        track.set_x(x)
        ############
        # END student code
        ############ 
        track.update_attributes(meas)
    
    def gamma(self, track, meas):
        ############
        # TODO Step 1: calculate and return residual gamma
        ############

        return (meas.z - meas.sensor.get_H(track.x))
        
        ############
        # END student code
        ############ 

    def S(self, track, meas, H):
        ############
        # TODO Step 1: calculate and return covariance of residual S
        ############

        return (H * track.P * H.transpose()) + meas.R
        
        ############
        # END student code
        ############ 