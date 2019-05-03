# -*- coding: utf-8 -*-
"""
Created on Tue Apr 16 09:45:56 2019

@author: lbrevaul
"""

from openmdao.api import Problem
import Launch_vehicle_Group
import numpy as np 
import sys
np.set_printoptions(threshold=sys.maxsize)
from matplotlib import pyplot as plt
import post_traitement
import constants as Cst
import specifications as Spec
import cma

"""
simu = 1
lowerbnd = np.array([265.,# chargement Mprop1
                         90., #chargement Mprop2
                         25., # theta_cmd_i
                         -15., # theta_cmd_f
                         -1.,# ksi
                         5., # Duree basculement
                         1., # Delta basculement
                         -10.,#delta azimut initial
                         15., #Mprop_boost
                         5., 5., 5.,  5.,  5.,#Command_alpha  
                         -5.,#Command_theta
                         0.6,#detar_boost
                         25.,25.,# cmd theta et 1 exo 
                         1.,# cmde phi dernier virage
                         3.,# alpha virage
                         30000., # altitude mise en incidence
                         5,5]) #cmd alpha_alignement

upperbnd = np.array([285,# chargement Mprop1
                         110.,#chargement Mprop2
                         40.,# theta_cmd_i
                         5.,# theta_cmd_f
                         1.,# ksis
                         20.,# Duree basculement
                         3.,# Delta basculement                         
                         0.,#delta azimut initial
                         30., #Mprop_boost
                         10.,  10.,  10., 10.,10.,#Command_alpha  
                         5.,#Command_theta 
                         .8,#detar_boost
                         55.,65.,# cmd theta et 1 exo 
                         2.2,# cmde phi dernier virage
                         10.,# alpha virage
                         40000., # altitude mise en incidence
                         20,20])#cmd_alpha_alignement


init = np.ones(len(upperbnd))*0.55
"""
liste_P = []
P_obj = Problem()
P_obj.model=Launch_vehicle_Group.Launcher_vehicle()

#Pb_0 = Problem()
#Pb_0.model=Launch_vehicle_Group.Launcher_vehicle()
#
#
#liste_P = [Pb_0]

Pb = P_obj 

def FPI(tuple_input):
#    x = tuple_input[0]
#    lowerbnd_exp = tuple_input[1]
#    upperbnd_exp = tuple_input[2]
    Pb = tuple_input
#    
    Pb.setup(check = False)
#    XX = lowerbnd_exp + (upperbnd_exp - lowerbnd_exp)*x
    Pb['Diameter_stage_1'] = 4.6
    Pb['Diameter_stage_2'] = 4.6
    Pb['Mass_flow_rate_stage_1'] = 219.
    Pb['Mass_flow_rate_stage_2'] = 219.
    Pb['Thrust_stage_1'] = 1000.
    Pb['Thrust_stage_2'] = 1150.
    Pb['OF_stage_1'] = 5.0
    Pb['OF_stage_2'] = 5.0
    Pb['N_eng_stage_1'] = 7.
    Pb['N_eng_stage_2'] = 1.
    Pb['Prop_mass_stage_1'] = 320000.
    Pb['Prop_mass_stage_2'] = 75000.
    Pb['NX_max_dim'] = 6.
    Pb['Pdyn_max_dim'] = 6.
    Pb['thetacmd_i'] = 2.72
    Pb['thetacmd_f'] = 10.
    Pb['ksi'] = 0.293
    Pb['Pitch_over_duration'] = 5.
    Pb['Exit_nozzle_area_stage_1'] = 0.5
    Pb['Exit_nozzle_area_stage_2'] = 0.5
    Pb['Delta_vertical_phase'] = 10.
    Pb['Delta_theta_pitch_over'] = 1.        
    Pb['command_stage_1_exo'] = np.array([10.,-2.])

    Pb.run_model()
        
        
    return Pb

P_out = FPI(Pb)
GLOW, contraintes = post_traitement.post_traitement(P_out)



Nb_final = np.int(P_out['Nb_pt_ascent'])

plt.figure()
plt.plot(P_out['T_ascent'][0:Nb_final],P_out['alt_ascent'][0:Nb_final]/1e3)
plt.xlabel('Time (s)')
plt.ylabel('Altitude (km)')

plt.figure()
plt.plot(P_out['T_ascent'][0:Nb_final],P_out['m_ascent'][0:Nb_final]/1e3)
plt.xlabel('Time (s)')
plt.ylabel('Mass (t)')

plt.figure()
plt.plot(P_out['T_ascent'][0:Nb_final],P_out['V_ascent'][0:Nb_final]/1e3)
plt.xlabel('Time (s)')
plt.ylabel('Velocity (km/s)')
