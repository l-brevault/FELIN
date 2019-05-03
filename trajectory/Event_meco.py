#import Constants as Cst
import constants as Cst

import numpy as np

def event_meco(t,x,parameters):
    
    mf = parameters['masses']['Mass_f']
    
    return x[4]- mf