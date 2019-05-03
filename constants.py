# -*- coding: utf-8 -*-
"""
Created on Wed Apr 24 10:46:18 2019

@author: lbrevaul
"""
import numpy as np
import constants as Cst

#Earth radius (m)
RT = 6378137.0

#Earth mass (kg)
MT =5.9736e24

#Standard acceleration
g0 = 9.80665

#Earth rotation velocity (rad / s)
Omega_T = 2.*np.pi/(86164.09)

#Gravity constant
G = 6.67408e-11
mu = 398600.4418e9

#Addition masses

#Initial state conditions
r0 = RT+100.
V0 = 10.
gamma0 = np.pi/2.


Duration_stage_separation = 2.

derating_stage_2 = 0.45
derating_stage_1 = 0.25

Pa_sol = 101325.0

coeff_losses = 0.1