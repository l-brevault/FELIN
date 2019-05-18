# -*- coding: utf-8 -*-
"""
Created on Fri Sep 28 13:13:17 2018

@author: mbalesde
"""

import constants as Cst
import numpy as np
import specifications as Spec
from trajectory.modele_atmos import compute_atmos


def event_command_stage_1_theta(t,x,parameters):
    
    r = x[0] 
    V = x[1]


    alt = r-Cst.RT
    (Temp,Pa,rho,c) = compute_atmos(alt)
    
    Pdyn =  .5*rho*V**2


    return Pdyn - Spec.specifications['command']['ascent']['pdyn_end_gravity_turn']