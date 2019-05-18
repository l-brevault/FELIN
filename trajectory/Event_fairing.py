#import Constants as Cst
import constants as Cst
import numpy as np 
from trajectory.modele_atmos import compute_atmos

def event_fairing_jettison(t, x,parameters):
    'an event is when f = 0 and event is increasing'

    r = x[0] 
    V = x[1]


    alt = r-Cst.RT
    (Temp,Pa,rho,c) = compute_atmos(alt)
    flux = 0.5*rho*V**3
    value = flux - 1135.
    #if flux is below 1135 (kg m^-2s^-1), jettison of the fairing

    return value

