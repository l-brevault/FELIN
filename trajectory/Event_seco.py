#import Constants as Cst
import constants as Cst

import numpy as np
import specifications as Spec

def event_seco(t, x, parameters):
    'an event is when f = 0 and event is increasing'

    r = x[0] 
    V = x[1]
    gamma = x[2]
    longi = x[3]
    m = x[4]

    lat = Spec.specifications['launch_site']['latitude']*np.pi/180.
    
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    sin_longi = np.sin(longi)
    cos_longi = np.cos(longi)
        
    P_TG_TGL= np.array([[-cos_longi*sin_lat, -sin_longi*sin_lat,cos_lat],\
                        [-sin_longi,cos_longi,0.],\
                        [-cos_longi*cos_lat,-sin_longi*cos_lat,-sin_lat]])

    pos_TGL = np.array([0.,0.,-r])
    pos_TG = np.matmul(P_TG_TGL.T,pos_TGL)
    
    vit_TGL = np.array([0.,V*np.cos(gamma),-V*np.sin(gamma)])
    vit_TG = np.matmul(P_TG_TGL.T,vit_TGL)
    
    e = 1/(Cst.mu)*((V*V-Cst.mu/r)*pos_TG - np.dot(pos_TG,vit_TG)*vit_TG)
    ecc = np.sqrt(e[0]**2+e[1]**2+e[2]**2)
    E = V*V/2.-Cst.mu/r
    a = -Cst.mu/(2*E)
    ra = a *(1+ecc)
    value = ra-Cst.RT - Spec.specifications['orbit']['altitude_apogee']
    return value

def event_seco_mass(t,x,parameters):
    
    mf2 = parameters['masses']['Mass_f']
    
    return x[4]- mf2