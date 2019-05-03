# -*- coding: utf-8 -*-
"""
Created on Wed Apr 24 10:46:18 2019

@author: lbrevaul
"""
import numpy as np

specifications = {}
from geographiclib.geodesic import Geodesic
geod = Geodesic.WGS84
#### Specifications de la mission ####

## Orbite
# Valeurs elements orbitaux
specifications['orbit']={}
specifications['orbit']['altitude_apogee'] = 800e3
specifications['orbit']['altitude_perigee'] = 800e3

specifications['orbit']['excentricite'] = 0.0
specifications['orbit']['perigee_min_transfer_orbit'] =  140e3

# Precisions
specifications['orbit']['precision_apogee'] = 1e3
specifications['orbit']['precision_excentricite'] = 0.05

specifications['mission'] = {}
specifications['mission']['Payload_mass'] = 6e3
## Command
specifications['command'] = {}
specifications['command']['ascent'] = {}
specifications['command']['ascent']['pdyn_end_gravity_turn']= 1e3
specifications['command']['ascent']['nx_max'] = 4.5
specifications['command']['ascent']['flux_max'] = 100.
specifications['command']['ascent']['pdyn_max'] = 40.
specifications['command']['ascent']['alpha_max'] = 15.

## Launch site
specifications['launch_site'] = {}
specifications['launch_site']['longitude']=  -52.78
specifications['launch_site']['latitude'] = 0.





specifications['masses_additionnelles_etage_2']={}
specifications['masses_additionnelles_etage_2']['SEL'] = 322
specifications['masses_additionnelles_etage_2']['LVA'] = 400
specifications['masses_additionnelles_etage_2']['Fairing_mass'] = 2000.

## Reserves perfo
specifications['reserves_perfos']={}
specifications['reserves_perfos']['etage_1']= 0.01 
specifications['reserves_perfos']['imbrules_etage_1']= 0.005 

specifications['reserves_perfos']['etage_2']= 0.02
specifications['reserves_perfos']['imbrules_etage_2']= 0.005
specifications['reserves_perfos']['desorbitation_etage_2']= 1000

