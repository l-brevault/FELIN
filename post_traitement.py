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
    r_end = Pb['r_ascent'][Nb_final] #altitude end simulation
    V_end = Pb['V_ascent'][Nb_final] #velocity end simulation
    gamma_end = Pb['gamma_ascent'][Nb_final]*np.pi/180. #flight path angle end simulation (rad)
    longi_end = Pb['longi_ascent'][Nb_final]*np.pi/180. #longitude end simulation (rad)
    m_end = Pb['m_ascent'][Nb_final] #mass end simulation

    lat_end = Spec.specifications['launch_site']['latitude']*np.pi/180.
    
    sin_lat = np.sin(lat_end)
    cos_lat = np.cos(lat_end)
    sin_longi = np.sin(longi_end)
    cos_longi = np.cos(longi_end)
        
    P_TG_TGL= np.array([[-cos_longi*sin_lat, -sin_longi*sin_lat,cos_lat],\
                        [-sin_longi,cos_longi,0.],\
                        [-cos_longi*cos_lat,-sin_longi*cos_lat,-sin_lat]])  #Matrix between TGL and TG frames

    pos_TGL = np.array([0.,0.,-r_end])        #position in TGL
    pos_TG = np.matmul(P_TG_TGL.T,pos_TGL)    #position in TG
    
    vit_TGL = np.array([0.,V_end*np.cos(gamma_end),-V_end*np.sin(gamma_end)]) #velocity in TGL
    vit_TG = np.matmul(P_TG_TGL.T,vit_TGL) #velocity in TG
    
    e = 1./(Cst.mu)*((V_end*V_end-Cst.mu/r_end)*pos_TG - np.dot(pos_TG,vit_TG)*vit_TG) #Eccentricity vector
    ecc = np.sqrt(e[0]**2+e[1]**2+e[2]**2) #Norm of eccentricity vector
    E = V_end*V_end/2.-Cst.mu/r_end #Orbit energy
    a = -Cst.mu/(2*E) #Semi axis
    ra = a *(1.+ecc) #Apogee radius
    rp = a *(1.-ecc)  #Perigee radius
    penal_apogee = (np.abs(ra-Cst.RT-Spec.specifications['orbit']['altitude_apogee'])-Spec.specifications['orbit']['precision_apogee'])/1e3
   
    ####---- Constraint on the perigee of the injection orbit
    penal_perigee = - (rp-Cst.RT-Spec.specifications['orbit']['perigee_min_transfer_orbit'])/1e3

    ####---- Constraint on the remaining propellant masses for orbit circularization

    if E+Cst.mu/(Spec.specifications['orbit']['altitude_apogee']+Cst.RT)<0.:
        penal_mass_prop = 1e5-(E+Cst.mu/(Spec.specifications['orbit']['altitude_apogee']+Cst.RT))
    else:
        V_apogee= np.sqrt(2*(E+Cst.mu/(Spec.specifications['orbit']['altitude_apogee']+Cst.RT))) #velocity at the apogee of the current injection orbit
        V_injection = np.sqrt(Cst.mu/(Spec.specifications['orbit']['altitude_apogee']+Cst.RT)) #velocity required for the circular target orbit
        delta_V = V_injection - V_apogee #deltaV to go from current orbit to circular orbit
        
        delta_M = m_end*(1.-1./np.exp(delta_V/(Cst.g0*Pb['Isp_stage_2'])))*Cst.coeff_losses #associated required mass for the deltaV manoeuver
            
        mass_remaining_prop_stage_2 = m_end-delta_M-Spec.specifications['mission']['Payload_mass']-Pb['Dry_mass_stage_2']
                
        penal_mass_prop = - mass_remaining_prop_stage_2[0]
            
    ####---- Constraint on Pdyn
    pdyn_max_ascent = np.max(Pb['pdyn_ascent'][0:Nb_final])
    penal_pdyn_ascent = pdyn_max_ascent/1e3-Spec.specifications['command']['ascent']['pdyn_max']

    ####---- Constraint on heat flux 
    
    flux_max_ascent = np.max(abs(Pb['flux_ascent'][0:Nb_final])/1000.)
    penal_flux_ascent =  flux_max_ascent/1e3-Spec.specifications['command']['ascent']['flux_max']

    #### ---- Gross-Lift-Off Weight and constraints
    
    GLOW = Pb['GLOW']
    
    if type(GLOW)== np.float64:
        GLOW = GLOW
    else :
        GLOW = GLOW[0]
        
    constraints_ascent = np.array([penal_alpha_ascent,\
                                   penal_nx_ascent,\
                                   penal_apogee,\
                                   penal_perigee/1e2,\
                                   penal_mass_prop/1e3,\
                                   penal_pdyn_ascent,\
                                   penal_flux_ascent/1e3])
    
    
      
    return GLOW, constraints_ascent

