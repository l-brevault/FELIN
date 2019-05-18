# -*- coding: utf-8 -*-
"""
Created on Thu Jul 19 13:31:11 2018

@author: lbrevaul
"""

import numpy as np
import constants as Cst
import specifications as Spec

def Command_stage_1(t,m,N_eng,Pa,Exit_nozzle_area,Pitch_over_duration,\
                 Delta_theta_pitch_over,Mass_f1,Delta_vertical_phase,\
                gamma,rho,V,mass_flow_rate_0,\
                 Isp,altitude,interp_theta_stage_1):

    #Constantes
    
    T = N_eng*(Isp*mass_flow_rate_0*Cst.g0-Pa*Exit_nozzle_area)
    mass_flow_rate = N_eng * mass_flow_rate_0
    alpha = 0.
    theta = 0.
    Duration_decay = 10.
    

    if t< Delta_vertical_phase : 
        #Vertical lift-off phase
        alpha = 0.
        theta = np.pi/2.
        
    elif ((t>=Delta_vertical_phase) and (t<Delta_vertical_phase + Pitch_over_duration)):    ###pitch over manoeuver
        
        theta = (gamma*180./np.pi - Delta_theta_pitch_over * (t - Delta_vertical_phase) / Pitch_over_duration)*np.pi/180.
        alpha = theta- gamma
        
    elif ((t>=Delta_vertical_phase + Pitch_over_duration) and (t<Delta_vertical_phase + Pitch_over_duration+ 3*Duration_decay)):    ###return to angle of attack = 0.
        theta = (gamma*180./np.pi - Delta_theta_pitch_over * np.exp(-(t - (Delta_vertical_phase + Pitch_over_duration)) / Duration_decay))*np.pi/180.
        alpha = theta- gamma
 
    elif (t>=Delta_vertical_phase + Pitch_over_duration+ 3.*Duration_decay):
        #gravity turn phase
        theta = gamma
        alpha = theta - gamma 
        
        #estimation of current NX and comparison to max spec NX
        NX = T/(m*Cst.mu/((altitude+Cst.RT)**2)) 
        Spec_NX = Spec.specifications['command']['ascent']['nx_max']-0.001
        if NX > Spec_NX:
            #modulation of thrust and mass flow rate to ensure NX<4.5
            T_NX =Spec_NX*(m*Cst.mu/((altitude+Cst.RT)**2))
            mass_flow_NX = (T_NX+Pa*Exit_nozzle_area*N_eng)/(N_eng*Cst.g0*Isp)
        
            if mass_flow_NX>Cst.derating_stage_1*mass_flow_rate_0:
            
                mass_flow_rate = mass_flow_NX*N_eng
                T =N_eng*(Cst.g0*Isp*mass_flow_rate/N_eng-Pa*Exit_nozzle_area)
            else:
                mass_flow_rate = Cst.derating_stage_1*mass_flow_rate*N_eng
                T =N_eng*(Cst.g0*Isp*mass_flow_rate/N_eng-Pa*Exit_nozzle_area)
           
        pdyn = .5*rho*V**2
        if pdyn<Spec.specifications['command']['ascent']['pdyn_end_gravity_turn']:
            if altitude<2000e3 and altitude>0.:
                #control phase out of the atmosphere
                theta = interp_theta_stage_1(altitude)*np.pi/180.
                alpha = theta - gamma
            else:
                theta = 0.*np.pi/180.
                alpha = theta - gamma   
        
                
    if (m<Mass_f1):
        mass_flow_rate = 0.
        T=0.
        
    return np.array([T, alpha, theta, mass_flow_rate])



