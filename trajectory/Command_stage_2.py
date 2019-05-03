# -*- coding: utf-8 -*-
"""
Created on Thu Jul 19 13:31:11 2018

@author: lbrevaul
"""

import constants as Cst
import numpy as np
import specifications as Spec

def Commande_Stage_2(t,m,N_eng,Pa,Exit_nozzle_area,
                      gamma,rho,V,mass_flow_rate_0,Mass_f2,
                      Isp, theta_i,theta_f,ksi,
                      tf2,altitude,duration_separation):

    T = 0.
    mass_flow_rate = 0.
    a = 100.
    tf2 = tf2+2.

    theta = np.arctan2((pow(a,ksi)*np.tan(theta_i*np.pi/180.)+(np.tan(theta_f*np.pi/180.)-pow(a,ksi)*np.tan(theta_i*np.pi/180.))*(t)/tf2),(pow(a,ksi)+(1.-pow(a,ksi))*(t)/tf2))   
    alpha =  theta-gamma
    
    #loi de poussee
    mass_flow_rate = N_eng * mass_flow_rate_0
    T = N_eng*(Cst.g0*Isp*mass_flow_rate_0-Pa*Exit_nozzle_area)
    
    NX = T/(m*Cst.mu/((altitude+Cst.RT)**2))
    
    Spec_NX = Spec.specifications['command']['ascent']['nx_max']-0.001
    if NX > Spec_NX:
        T_NX = Spec_NX*(m*Cst.mu/((altitude+Cst.RT)**2))
        
        debit_NX = (T_NX+Pa*Exit_nozzle_area*N_eng)/(N_eng*Cst.g0*Isp)  
        
        if debit_NX>Cst.derating_stage_2*mass_flow_rate_0:
            
            mass_flow_rate = debit_NX
            T =N_eng*(Cst.g0*Isp*mass_flow_rate-Pa*Exit_nozzle_area)
        else:
            mass_flow_rate = Cst.derating_stage_2*mass_flow_rate
            T =N_eng*(Cst.g0*Isp*mass_flow_rate-Pa*Exit_nozzle_area)
            
   
    if (t <= duration_separation) or (m<Mass_f2):
        mass_flow_rate = 0.
        #alpha = 0.
        #theta = gamma
        T=0.
        
   
    return np.array([T, alpha, theta, mass_flow_rate])


