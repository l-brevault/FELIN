"""
Created on Mon Oct 30 11:24:33 2017

@author: lbrevaul
"""

import numpy as np
from openmdao.api import Group, IndepVarComp
import propulsion.Propulsion as Propulsion
import trajectory.Trajectory as Trajectory
import structure.Dry_Mass_stage_1 as Dry_Mass_stage_1 
import structure.Dry_Mass_stage_2 as Dry_Mass_stage_2
import aerodynamics.Aerodynamics as Aerodynamics

class Launcher_vehicle(Group):
           
    def setup(self):
        
        
        indeps = self.add_subsystem('indeps', IndepVarComp(), promotes=['*'])
        indeps.add_output('Diameter_stage_1',4.6)
        indeps.add_output('Diameter_stage_2',4.6)
        indeps.add_output('Mass_flow_rate_stage_1',219.)
        indeps.add_output('Mass_flow_rate_stage_2',219.)
        indeps.add_output('Thrust_stage_1',1000.)
        indeps.add_output('Thrust_stage_2',1150.)
        indeps.add_output('Pc_stage_1',100.)
        indeps.add_output('Pc_stage_2',100.)
        indeps.add_output('Pe_stage_1',1.)
        indeps.add_output('Pe_stage_2',1.)
        indeps.add_output('OF_stage_1',2.7)
        indeps.add_output('OF_stage_2',2.7)
        indeps.add_output('N_eng_stage_1',7.)
        indeps.add_output('N_eng_stage_2',1.)
        indeps.add_output('Prop_mass_stage_1',350000.)
        indeps.add_output('Prop_mass_stage_2',75000.)
        indeps.add_output('Pdyn_max_dim',40.)
        indeps.add_output('thetacmd_i',2.72)
        indeps.add_output('thetacmd_f',20.)
        indeps.add_output('ksi',0.293)
        indeps.add_output('Pitch_over_duration',5.)
        indeps.add_output('Exit_nozzle_area_stage_1',0.5)
        indeps.add_output('Exit_nozzle_area_stage_2',0.5)
        indeps.add_output('Delta_vertical_phase',10.)
        indeps.add_output('Delta_theta_pitch_over',1.)
        indeps.add_output('is_fallout',1.)        
        
        indeps.add_output('command_stage_1_exo',np.array([1.,1.]))

        
        cycle = self.add_subsystem('cycle', Group(), promotes=['*'])
        
        

        Propu = cycle.add_subsystem('Propu', Propulsion.Propulsion_Comp(),promotes_inputs=['Pc_stage_1','Pe_stage_1',
                                    'OF_stage_1','Pc_stage_2','Pe_stage_2','OF_stage_2'],
                                    promotes_outputs=['Isp_stage_1','Isp_stage_2'])
           
        Struct_1 = cycle.add_subsystem('Struct_1', Dry_Mass_stage_1.Dry_Mass_stage_1_Comp(),
                                       promotes_inputs=['Diameter_stage_1','OF_stage_1',
                                    'N_eng_stage_1','Diameter_stage_2','Isp_stage_1','Prop_mass_stage_1',
                                    'Thrust_stage_1','Pdyn_max_dim'],
                                    promotes_outputs=['Dry_mass_stage_1'])
        
        Struct_2 = cycle.add_subsystem('Struct_2', Dry_Mass_stage_2.Dry_Mass_stage_1_Comp(),
                                       promotes_inputs=['Prop_mass_stage_2'],promotes_outputs=['Dry_mass_stage_2'])

        Aero = cycle.add_subsystem('Aero', Aerodynamics.Aerodynamics_Comp(),promotes_outputs=['Table_CX_complete_ascent',
                                   'Mach_table','AoA_table','CX_fallout_stage_1','CZ_fallout_stage_1',])

        Traj = cycle.add_subsystem('Traj', Trajectory.Trajectory_comp(),
                                   promotes_inputs=['Diameter_stage_1','Diameter_stage_2','Mass_flow_rate_stage_1',
                                                    'Mass_flow_rate_stage_2','N_eng_stage_1','N_eng_stage_2',
                                                    'OF_stage_1','OF_stage_2','Isp_stage_1','Isp_stage_2',
                                                    'Prop_mass_stage_1','Prop_mass_stage_2','Dry_mass_stage_1',
                                                    'Dry_mass_stage_2','Pitch_over_duration','thetacmd_i',
                                                    'thetacmd_f','ksi','Exit_nozzle_area_stage_1','Exit_nozzle_area_stage_2',
                                                    'Delta_vertical_phase','Delta_theta_pitch_over','Table_CX_complete_ascent',
                                                    'Mach_table','AoA_table','command_stage_1_exo','CX_fallout_stage_1','CZ_fallout_stage_1','is_fallout'],
                                   promotes_outputs=['T_ascent','alt_ascent','flux_ascent','r_ascent',
                                                     'V_ascent','theta_ascent','alpha_ascent','nx_ascent','alpha_cont',
                                                     'Nb_pt_ascent','m_ascent','CX_ascent','GLOW',
                                                     'lat_ascent','gamma_ascent','longi_ascent','thrust_ascent',
                                                     'mass_flow_rate_ascent','Mach_ascent','pdyn_ascent',
                                                     'rho_ascent','distance_ascent','state_separation_stage_1',
                                                     'max_pdyn_load_ascent_stage_1',\
                                                     'T_fallout','alt_fallout','flux_fallout','r_fallout',
                                                     'V_fallout','theta_fallout','alpha_fallout','nx_fallout',
                                                     'Nb_pt_fallout','m_fallout','CX_fallout',
                                                     'lat_fallout','gamma_fallout','longi_fallout','thrust_fallout',
                                                     'mass_flow_rate_fallout','Mach_fallout','pdyn_fallout',
                                                     'rho_fallout','distance_fallout',])
    
          
        

