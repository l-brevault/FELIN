#import Constants as Cst
import constants as Cst

import numpy as np
import specifications as Spec

def event_seco(t, x, parameters):
    'an event is when f = 0 and event is increasing'

    r = x[0] #altitude (m)
    V = x[1] #velocity (m/s)
    gamma = x[2] #flight path angle (rad)
    longi = x[3] #longitude (rad)
    m = x[4] #launch vehicle mass (kg)

    lat = Spec.specifications['launch_site']['latitude']*np.pi/180.
    
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    sin_longi = np.sin(longi)
    cos_longi = np.cos(longi)
        
    P_TG_TGL= np.array([[-cos_longi*sin_lat, -sin_longi*sin_lat,cos_lat],\
                        [-sin_longi,cos_longi,0.],\
                        [-cos_longi*cos_lat,-sin_longi*cos_lat,-sin_lat]])  #matrix for TGL and TG transformation

    pos_TGL = np.array([0.,0.,-r]) #position in TGL
    pos_TG = np.matmul(P_TG_TGL.T,pos_TGL) #position in TG
    
    vit_TGL = np.array([0.,V*np.cos(gamma),-V*np.sin(gamma)]) #velocity in TGL
    vit_TG = np.matmul(P_TG_TGL.T,vit_TGL) #velocity in TG
    
    e = 1/(Cst.mu)*((V*V-Cst.mu/r)*pos_TG - np.dot(pos_TG,vit_TG)*vit_TG) #orbit eccentricity vector
    ecc = np.sqrt(e[0]**2+e[1]**2+e[2]**2) #orbit eccentricity norm
    E = V*V/2.-Cst.mu/r #orbit energy
    a = -Cst.mu/(2*E) #orbit semi axis
    ra = a *(1+ecc) #altitude apogee
    value = ra-Cst.RT - Spec.specifications['orbit']['altitude_apogee']  #comparison of current apogee and target apogee
    #integration ends if on the current orbit, the apogee = target apogee
    return value

def event_seco_mass(t,x,parameters):
    
    mf2 = parameters['masses']['Mass_f']
    
    return x[4]- mf2