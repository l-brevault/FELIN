# -*- coding: utf-8 -*-
"""
Created on Fri Oct  5 13:46:06 2018

@author: mbalesde
"""
import numpy as np
import constants as Cst

def event_impact(t,x,parameters):
    'an event is when f = 0 and event is increasing'

    r = x[0] 
    alt = r-Cst.RT


    value= alt-10.

    return value
