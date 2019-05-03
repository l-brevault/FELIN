# -*- coding: utf-8 -*-
"""
Created on Wed Apr 24 10:46:18 2019

@author: lbrevaul
"""

import numpy as np
import constants as Cst

import specifications as Spec
def post_traitement(Pb):
     
    Nb_final = np.int(Pb['Nb_pt_ascent']-1)

    
    ####---- Constraint on ascent for maximal angle of attack
    alpha_max = Spec.specifications['command']['ascent']['alpha_max']
    alpha = Pb['alpha_cont'][0]
    penal_alpha_ascent = alpha- alpha_max
     
        
    ####---- Constraint on the maximal axial load factor for the payload
    Nx_target = Spec.specifications['command']['ascent']['nx_max'] 
    NX_max_flight = np.max(Pb['nx_ascent'][0:Nb_final])
    penal_nx_ascent = NX_max_flight-Nx_target

        
    ####---- Constraint on the apogee of the injection orbit
    r_end = Pb['r_ascent'][Nb_final]
    V_end = Pb['V_ascent'][Nb_final]
    gamma_end = Pb['gamma_ascent'][Nb_final]*np.pi/180.
    longi_end = Pb['longi_ascent'][Nb_final]*np.pi/180.
    m_end = Pb['m_ascent'][Nb_final]

    lat_end = Spec.specifications['launch_site']['latitude']*np.pi/180.
    
    sin_lat = np.sin(lat_end)
    cos_lat = np.cos(lat_end)
    sin_longi = np.sin(longi_end)
    cos_longi = np.cos(longi_end)
        
    P_TG_TGL= np.array([[-cos_longi*sin_lat, -sin_longi*sin_lat,cos_lat],\
                        [-sin_longi,cos_longi,0.],\
                        [-cos_longi*cos_lat,-sin_longi*cos_lat,-sin_lat]])

    pos_TGL = np.array([0.,0.,-r_end])
    pos_TG = np.matmul(P_TG_TGL.T,pos_TGL)
    
    vit_TGL = np.array([0.,V_end*np.cos(gamma_end),-V_end*np.sin(gamma_end)])
    vit_TG = np.matmul(P_TG_TGL.T,vit_TGL)
    
    e = 1./(Cst.mu)*((V_end*V_end-Cst.mu/r_end)*pos_TG - np.dot(pos_TG,vit_TG)*vit_TG)
    ecc = np.sqrt(e[0]**2+e[1]**2+e[2]**2)
    E = V_end*V_end/2.-Cst.mu/r_end
    a = -Cst.mu/(2*E)
    ra = a *(1.+ecc) 
    rp = a *(1.-ecc)  
    penal_apogee = (np.abs(ra-Cst.RT-Spec.specifications['orbit']['altitude_apogee'])-Spec.specifications['orbit']['precision_apogee'])/1e3
   
    ####---- Constraint on the perigee of the injection orbit
    penal_perigee = - (rp-Cst.RT-Spec.specifications['orbit']['perigee_min_transfer_orbit'])/1e3

    ####---- Constraint on the remaining propellant masses

    E = V_end*V_end/2.-Cst.mu/r_end
    if E+Cst.mu/(Spec.specifications['orbit']['altitude_apogee']+Cst.RT)<0.:
        penal_mass_prop = 1e5-(E+Cst.mu/(Spec.specifications['orbit']['altitude_apogee']+Cst.RT))
    else:
        V_apogee= np.sqrt(2*(E+Cst.mu/(Spec.specifications['orbit']['altitude_apogee']+Cst.RT)))
        V_injection = np.sqrt(Cst.mu/(Spec.specifications['orbit']['altitude_apogee']+Cst.RT))
        delta_V = V_injection - V_apogee
        
        delta_M = m_end*(1.-1./np.exp(delta_V/(Cst.g0*Pb['Isp_stage_2'])))*Cst.coeff_losses
            
        mass_remaining_prop_stage_2 = m_end-delta_M-Spec.specifications['mission']['Payload_mass']-Pb['Dry_mass_stage_2']
                
        penal_mass_prop = - mass_remaining_prop_stage_2[0]
            
    ####---- Constraint on Pdyn
    pdyn_max_ascent = np.max(Pb['pdyn_ascent'][0:Nb_final])
    
    penal_pdyn_ascent = pdyn_max_ascent/1e3-Spec.specifications['command']['ascent']['pdyn_max']

    ####---- Constraint on heat flux 
    
    flux_max_ascent = np.max(abs(Pb['flux_ascent'][0:Nb_final])/1000.)
    penal_flux_ascent =  flux_max_ascent/1e3-Spec.specifications['command']['ascent']['flux_max']

    #### ---- Gross-Lift-Off Weight
    
    GLOW = Pb['GLOW']
    
    if type(GLOW)== np.float64:
        GLOW = GLOW
    else :
        GLOW = GLOW[0]
        
    contraintes_montee = np.array([penal_alpha_ascent,\
                                   penal_nx_ascent,\
                                   penal_apogee,\
                                   penal_perigee/1e2,\
                                   penal_mass_prop/1e3,\
                                   penal_pdyn_ascent,\
                                   penal_flux_ascent/1e3])
    
    
      
    return GLOW, contraintes_montee

