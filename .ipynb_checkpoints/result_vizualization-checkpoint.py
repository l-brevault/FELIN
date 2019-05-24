# -*- coding: utf-8 -*-
"""
Created on Wed Apr 24 10:46:18 2019

@author: lbrevaul
"""

import numpy as np
from matplotlib import pyplot as plt
import pandas as pd

def plots_output(P_out):
    Nb_final = np.int(P_out['Nb_pt_ascent'])
    
    f, axs = plt.subplots(3,2,figsize=(15,15))
    
    plt.subplot(421)
    plt.plot(P_out['T_ascent'][0:Nb_final],P_out['alt_ascent'][0:Nb_final]/1e3)
    plt.xlabel('Time (s)')
    plt.ylabel('Altitude (km)')
    
    plt.subplot(422)
    plt.plot(P_out['T_ascent'][0:Nb_final],P_out['V_ascent'][0:Nb_final]/1e3)
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (km/s)')
    
    plt.subplot(423)
    plt.plot(P_out['T_ascent'][0:Nb_final],P_out['m_ascent'][0:Nb_final]/1e3)
    plt.xlabel('Time (s)')
    plt.ylabel('Mass (t)')
    
    plt.subplot(424)
    plt.plot(P_out['T_ascent'][0:Nb_final],P_out['gamma_ascent'][0:Nb_final])
    plt.xlabel('Time (s)')
    plt.ylabel('Flight path angle (deg)')
    
    plt.subplot(425)
    plt.plot(P_out['T_ascent'][0:Nb_final],P_out['theta_ascent'][0:Nb_final])
    plt.xlabel('Time (s)')
    plt.ylabel('Pitch angle (deg)')
    
    plt.subplot(426)
    plt.plot(P_out['T_ascent'][0:Nb_final],P_out['nx_ascent'][0:Nb_final])
    plt.xlabel('Time (s)')
    plt.ylabel('Load factor')
    
    plt.subplot(427)
    plt.plot(P_out['T_ascent'][0:Nb_final],P_out['pdyn_ascent'][0:Nb_final]/1e3)
    plt.xlabel('Time (s)')
    plt.ylabel('Dynamic Pressure (kPa)')
    
    plt.subplot(428)
    plt.plot(P_out['longi_ascent'][0:Nb_final],P_out['lat_ascent'][0:Nb_final])
    plt.xlabel('Longitude (deg)')
    plt.ylabel('Latitude (deg)')
    
    plt.show()
    
    # Fallout
    if P_out['is_fallout'] == 1.:
        print('Plot of fallout phase')
        Nb_final_fallout = np.int(P_out['Nb_pt_fallout'])

        f, axs = plt.subplots(3,2,figsize=(15,15))

        plt.subplot(421)
        plt.plot(P_out['T_fallout'][0:Nb_final_fallout],P_out['alt_fallout'][0:Nb_final_fallout]/1e3)
        plt.xlabel('Time (s)')
        plt.ylabel('Altitude (km)')

        plt.subplot(422)
        plt.plot(P_out['T_fallout'][0:Nb_final_fallout],P_out['V_fallout'][0:Nb_final_fallout]/1e3)
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity (km/s)')

        plt.subplot(423)
        plt.plot(P_out['T_fallout'][0:Nb_final_fallout],P_out['m_fallout'][0:Nb_final_fallout]/1e3)
        plt.xlabel('Time (s)')
        plt.ylabel('Mass (t)')

        plt.subplot(424)
        plt.plot(P_out['T_fallout'][0:Nb_final_fallout],P_out['gamma_fallout'][0:Nb_final_fallout])
        plt.xlabel('Time (s)')
        plt.ylabel('Flight path angle (deg)')

        plt.subplot(425)
        plt.plot(P_out['T_fallout'][0:Nb_final_fallout],P_out['theta_fallout'][0:Nb_final_fallout])
        plt.xlabel('Time (s)')
        plt.ylabel('Pitch angle (deg)')

        plt.subplot(426)
        plt.plot(P_out['T_fallout'][0:Nb_final_fallout],P_out['nx_fallout'][0:Nb_final_fallout])
        plt.xlabel('Time (s)')
        plt.ylabel('Load factor')

        plt.subplot(427)
        plt.plot(P_out['T_fallout'][0:Nb_final_fallout],P_out['pdyn_fallout'][0:Nb_final_fallout]/1e3)
        plt.xlabel('Time (s)')
        plt.ylabel('Dynamic Pressure (kPa)')

        plt.subplot(428)
        plt.plot(P_out['longi_fallout'][0:Nb_final_fallout],P_out['lat_fallout'][0:Nb_final_fallout])
        plt.xlabel('Longitude (deg)')
        plt.ylabel('Latitude (deg)')

        plt.show()
    
    
    
    
def result_table(P_out):
    
    data = [[P_out['GLOW'][0]/1e3,
         P_out['Dry_mass_stage_1'][0]/1e3,
         P_out['Dry_mass_stage_2'][0]/1e3,
        P_out['Prop_mass_stage_1'][0]/1e3,
        P_out['Prop_mass_stage_2'][0]/1e3]]
    df = pd.DataFrame(data, columns=["GLOW (t)", "Dry_mass_stage_1  (t)", 
                                     "Dry_mass_stage_2  (t)", "Prop_mass_stage_1 (t)", 
                                     "Prop_mass_stage_2 (t)"])
    
    return [data, df]