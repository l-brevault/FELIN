# -*- coding: utf-8 -*-
"""
Created on Wed Apr 24 10:46:18 2019

@author: lbrevaul
"""
import numpy as np

specifications = {}
#### Specifications of the mission ####

# Target orbit
specifications['orbit']={}
specifications['orbit']['altitude_apogee'] = 700e3 # in m
specifications['orbit']['altitude_perigee'] = 700e3 # in m
specifications['orbit']['perigee_min_transfer_orbit'] =  140e3 # in m
specifications['orbit']['precision_apogee'] = 1e3 # Precisions on the apogee orbit injection, in m

# Mission payload mass
specifications['mission'] = {}
specifications['mission']['Payload_mass'] = 5e3 # in kg

## Max loads during ascent flight
specifications['command'] = {}
specifications['command']['ascent'] = {}
specifications['command']['ascent']['pdyn_end_gravity_turn']= 1e3 # in Pa
specifications['command']['ascent']['nx_max'] = 4.5
specifications['command']['ascent']['flux_max'] = 100. # in W/m2
specifications['command']['ascent']['pdyn_max'] = 40. # in kPa
specifications['command']['ascent']['alpha_max'] = 15. # in deg

## Launch site caracteristics
specifications['launch_site'] = {}
specifications['launch_site']['longitude']=  -52.78 # in deg
specifications['launch_site']['latitude'] = 0.

## Stage 2 additional mass definition
specifications['stage_2_additional_masses'] = {}
specifications['stage_2_additional_masses']['Fairing_mass'] = 2000. # in kg



