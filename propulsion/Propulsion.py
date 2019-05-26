# -*- coding: utf-8 -*-
"""
Created on Wed Mar 27 09:06:25 2019

@author: lbrevaul
"""

from __future__ import print_function
import numpy as np
from openmdao.api import ExplicitComponent
import sys
from scipy.interpolate import interp1d
sys.path.append(r'../')


class Propulsion_Comp(ExplicitComponent):
    def setup(self):
        ###Inputs definition
        self.add_input('Pc_stage_1', val=100.0,desc = 'Chamber pressure in bar - stage 1')
        self.add_input('Pe_stage_1', val=1.0,desc = 'Exit nozzle pressure in bar - stage 1')
        self.add_input('OF_stage_1', val=3.0,desc = 'Mixture ratio between oxidizer and fuel - stage 1')

        self.add_input('Pc_stage_2', val=100.0,desc = 'Chamber pressure in bar - stage 2')
        self.add_input('Pe_stage_2', val=1.0,desc = 'Exit nozzle pressure in bar - stage 2')
        self.add_input('OF_stage_2', val=3.0,desc = 'Mixture ratio between oxidizer and fuel - stage 2')


        ###Output definition        
        self.add_output('Isp_stage_1' ,val=330.,  desc = 'Specific impusle - stage 1')
        self.add_output('Isp_stage_2' ,val=330.,  desc = 'Specific impusle - stage 2')
        
    def compute(self, inputs, outputs):

        Mixture_ratio_tab = np.array([2.,3.,4.,5.,6.])
        Molecular_mass_tab = np.array([6.048,8.055,10.007,11.83,13.48])  #molecular mass at combustion for the couple LOX/LH2
        Gamma_t_tab = np.array([1.2949,1.2442,1.2004,1.1646,1.1415]) #isentropic coefficient at the throat for the couple LOX/LH2
        Flame_temperature_tab = np.array([1797.7,2451.63,2950.72,3293.24,3499.67]) #flame temperature at combustion for the couple LOX/LH2
        
        f_Molecular_mass = interp1d(Mixture_ratio_tab, Molecular_mass_tab, kind='cubic') #cubic interpolator 1d
        f_Gamma_t = interp1d(Mixture_ratio_tab, Gamma_t_tab, kind='cubic') #cubic interpolator 1d
        f_Flame_temperature = interp1d(Mixture_ratio_tab, Flame_temperature_tab, kind='cubic') #cubic interpolator 1d
        
        Molecular_mass_stage_1 = f_Molecular_mass(inputs['OF_stage_1'])
        Molecular_mass_stage_2 = f_Molecular_mass(inputs['OF_stage_2'])
        Gamma_t_stage_1 = f_Gamma_t(inputs['OF_stage_1'])
        Gamma_t_stage_2 = f_Gamma_t(inputs['OF_stage_2'])
        Flame_temperature_stage_1 = f_Flame_temperature(inputs['OF_stage_1'])
        Flame_temperature_stage_2 = f_Flame_temperature(inputs['OF_stage_2'])

        eta_c_1 = 0.92  #efficiency combustion
        eta_c_2 = 0.95  #efficiency combustion
        lambda_1 = 0.92  #efficiency nozzle
        lambda_2 = 0.95  #efficiency nozzle
       
        g0 = 9.80665  #Earth gravity constant
        R_stage_1 = 8314./Molecular_mass_stage_1 #gas constant
        R_stage_2 = 8314./Molecular_mass_stage_2 #gas constant
        
        # Calculation characteristic velocity
        C_star_stage_1 = eta_c_1*(np.sqrt(Gamma_t_stage_1*R_stage_1*Flame_temperature_stage_1))/(Gamma_t_stage_1*(2/(Gamma_t_stage_1+1))**((Gamma_t_stage_1+1)/(2*(Gamma_t_stage_1-1))))
        C_star_stage_2 = eta_c_2*(np.sqrt(Gamma_t_stage_2*R_stage_2*Flame_temperature_stage_2))/(Gamma_t_stage_2*(2/(Gamma_t_stage_2+1))**((Gamma_t_stage_2+1)/(2*(Gamma_t_stage_2-1))))

        # Calculation nozzle aera ratio        
        eps_stage_1 = (2./(Gamma_t_stage_1+1.)**(1./(Gamma_t_stage_1-1.)))*(inputs['Pc_stage_1']/inputs['Pe_stage_1'])**(1./Gamma_t_stage_1)/np.sqrt(((Gamma_t_stage_1+1.)/(Gamma_t_stage_1-1.))*(1.-(inputs['Pe_stage_1']/inputs['Pc_stage_1'])**((Gamma_t_stage_1-1.)/(Gamma_t_stage_1))))
        eps_stage_2 = (2./(Gamma_t_stage_2+1.)**(1./(Gamma_t_stage_2-1.)))*(inputs['Pc_stage_2']/inputs['Pe_stage_2'])**(1./Gamma_t_stage_2)/np.sqrt(((Gamma_t_stage_2+1.)/(Gamma_t_stage_2-1.))*(1.-(inputs['Pe_stage_2']/inputs['Pc_stage_2'])**((Gamma_t_stage_2-1.)/(Gamma_t_stage_2))))

        # Calculation specific impulse
        Isp_stage_1 = lambda_1*(C_star_stage_1/g0)*(Gamma_t_stage_1*np.sqrt((2./(Gamma_t_stage_1-1.))*(2./(Gamma_t_stage_1+1.))**((Gamma_t_stage_1+1.)/(Gamma_t_stage_1-1.))*(1.-inputs['Pe_stage_1']/inputs['Pc_stage_1'])**((Gamma_t_stage_1-1.)/Gamma_t_stage_1))+eps_stage_1/inputs['Pc_stage_1']*(inputs['Pe_stage_1']))
        Isp_stage_2 = lambda_2*(C_star_stage_2/g0)*(Gamma_t_stage_2*np.sqrt((2./(Gamma_t_stage_2-1.))*(2./(Gamma_t_stage_2+1.))**((Gamma_t_stage_2+1.)/(Gamma_t_stage_2-1.))*(1.-inputs['Pe_stage_2']/inputs['Pc_stage_2'])**((Gamma_t_stage_2-1.)/Gamma_t_stage_2))+eps_stage_2/inputs['Pc_stage_2']*(inputs['Pe_stage_2']))
        
                
        outputs['Isp_stage_1'] = Isp_stage_1      
            
        outputs['Isp_stage_2'] = Isp_stage_2
