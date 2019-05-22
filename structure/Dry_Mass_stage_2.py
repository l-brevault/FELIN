# -*- coding: utf-8 -*-
"""
Created on Wed Mar 01 15:08:27 2017

@author: lbrevaul
"""

from __future__ import print_function
import numpy as np
from openmdao.api import ExplicitComponent


class Dry_Mass_stage_1_Comp(ExplicitComponent):
    def setup(self):
        self.add_input('Prop_mass_stage_2',val=1.)
		
        self.add_output('Dry_mass_stage_2',val=3000.)

    def compute(self, inputs, outputs):
        
        #Regression to estimate the dry mass (without the propellant) of the second stage as a function of the propellant mass
        outputs['Dry_mass_stage_2'] = (55.*(inputs['Prop_mass_stage_2']/1e3)**(-0.49))/100*inputs['Prop_mass_stage_2'] ## Transcost MODEL
