# -*- coding: utf-8 -*-
"""
Created on Thu Jul 19 13:31:11 2018

@author: lbrevaul
"""

import numpy as np

import constants as Cst
import specifications as Spec
import geopy.distance as dist
from trajectory.modele_atmos import compute_atmos

def simulation_fallout_stage_1(t,x, Parameters):


    Integration = Parameters['simu']['Mode_simu']
    # this function is volontary empty

    if Integration == 1.:
        return np.zeros([1,5])
    else:
        return (0., 0., 0., 0., 0., 0.,
                0.,0.,0.,
                0.,0.,0.,
                0.,0.,0.,0.,0.,0.,0.)
 
    '''
    # to be included at the end of the implemented function
    if alt<0. :
        dx = np.zeros([1,5])
    
    if Integration == 1.:
         #during integration just return the current state
        return dx[0]
    
    else:
        #during post traitment return all the required data to be saved
        #Load factors to save data: Pdyn, Flux, Distance    
        Pdyn = 0.5*rho*V**2
        flux= Pdyn*V
        lat = Spec.specifications['launch_site']['latitude']
        
        distance = dist.great_circle((Spec.specifications['launch_site']['latitude'],Spec.specifications['launch_site']['longitude']),\
                                     (lat,longi*180/np.pi+Spec.specifications['launch_site']['longitude'])).km
     
        CA = CX*np.cos(np.abs(alpha))+CZ*np.sin(np.abs(alpha))  #aerodynamical force
        NX = (thrust-Pdyn*CA*S_ref)/(m*Cst.mu/((alt+Cst.RT)**2))  #axial load factor

        return (r, V, gamma, longi, m, NX,
                Mach,Pdyn,flux,
                alt,alpha,theta,
                rho, CX,CZ,thrust,Mass_flow_rate,distance,lat)
	'''
