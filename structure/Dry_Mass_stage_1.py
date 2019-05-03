# -*- coding: utf-8 -*-
"""
Created on Thu Jul 19 13:31:11 2018

@author: lbrevaul
"""
from __future__ import print_function
import numpy as np
from openmdao.api import ExplicitComponent
from scipy import integrate
from subprocess import Popen
import structure.Mass_models as mmf
import specifications as Spec


class Dry_Mass_stage_1_Comp(ExplicitComponent):
    def setup(self):

        ###Inputs definition
        self.add_input('Diameter_stage_1', val=3.0)
        self.add_input('OF_stage_1',val=1.)
        self.add_input('N_eng_stage_1', val=1.)
        self.add_input('Diameter_stage_2',val=1.)
        self.add_input('Isp_stage_1',val=1.)
        self.add_input('Prop_mass_stage_1',val=1.)
        self.add_input('Thrust_stage_1',val=1.)
        self.add_input('Pdyn_max_dim',val=1.)
        
        
        ###Output definition       
        self.add_output('Dry_mass_stage_1',val=3000.)
        
    def compute(self, inputs, outputs):
        
        Constants={}
        
        Constants['Type_prop']='Cryogenic'
        Constants['Engine_cycle']='SC'
        Constants['P_tank_Ox'] = 3.0
        Constants['P_tank_F'] = 3.0
        Constants['Tank_config']='intertank'
        Constants['Stage_in_staging']='lower'
        Constants['Type_struct_intertank']='Al'
        Constants['Techno_TVC']='electromechanic'
        Constants['Thrust_frame_material']='Al'
        Constants['Redundancy_level'] = 3
        Constants['Type_fuel']='LH2'
        Constants['S_interstage']=0.0
        Constants['g0']=9.80665
        Constants['Type_interstage']='lower'
        Constants['SSM_TF_1st_stage'] = 1.25
        Constants['Masse_aux_stage_1'] = 9000.
        Constants['NX_max_dim'] = Spec.specifications['command']['ascent']['nx_max']
        
        Thrust_1 = inputs['Thrust_stage_1']*1e3       
        Total_Thrust= inputs['N_eng_stage_1']*Thrust_1
         
        
        #Sizing tanks
        S_Ox,S_F,S_totale,S_dome,S_exterieur,L_total = mmf.sizing(inputs['Prop_mass_stage_1'], inputs['OF_stage_1'], inputs['Diameter_stage_1'],Constants['Type_fuel'])
        
        #M_engine (MER)
        M_engine = mmf.engine_mass(Thrust_1, Constants['Type_prop'],Constants['Engine_cycle'])
        M_eng = inputs['N_eng_stage_1']*M_engine

        #M thrust frame
        M_thrust_frame = mmf.thrust_frame_mass(Total_Thrust/inputs['N_eng_stage_1']/1000,M_eng/inputs['N_eng_stage_1'],Constants['NX_max_dim'],inputs['N_eng_stage_1'],Constants['Thrust_frame_material'],Constants['SSM_TF_1st_stage'])
        
        #M tanks
        M_F = inputs['Prop_mass_stage_1']/(1+inputs['OF_stage_1'])
        M_OX = inputs['Prop_mass_stage_1'] - M_F
        mu_LOX = 1141.
        if Constants['Type_fuel'] == 'LH2':
        	mu_F = 70.85
        elif Constants['Type_fuel'] =='CH4':
        	mu_F = 422.36
        elif Constants['Type_fuel'] == 'RP1':
        	mu_F = 810. 
        
        
        V_Ox = M_OX / mu_LOX
        V_FT = M_F / mu_F
        M_FT,M_OxT,M_TPS_OxT,M_TPS_FT,M_intertank = mmf.tank_mass(inputs['Pdyn_max_dim'],Constants['NX_max_dim'],Constants['P_tank_Ox'],Constants['P_tank_F'],V_FT,V_Ox,inputs['Diameter_stage_1'],S_Ox,S_F,2*S_dome,S_totale,Constants['Type_prop'],Constants['Tank_config'],Constants['Stage_in_staging'],Constants['Type_struct_intertank'])
                    
        # M TVC
        M_TVC = inputs['N_eng_stage_1']*mmf.TVC_mass(Thrust_1,Constants['Techno_TVC'])
        
        #M avio, M EPS
        M_avio, M_EPS = mmf.EPS_avio_mass(S_exterieur,Constants['Redundancy_level'])
        	
        #M_interstage
        M_interstage = mmf.mass_interstage(Constants['S_interstage'],inputs['Diameter_stage_1'],inputs['Diameter_stage_2'],Constants['Type_interstage'])
           
        
        Dry_mass_stage_1 = M_eng+M_thrust_frame+M_FT+M_OxT+M_TPS_OxT+\
                        M_TPS_FT+M_TVC+M_avio+M_EPS+M_intertank+\
                        M_interstage+Constants['Masse_aux_stage_1']
                        
        outputs['Dry_mass_stage_1']=Dry_mass_stage_1
        
