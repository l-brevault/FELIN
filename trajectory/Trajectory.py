# -*- coding: utf-8 -*-
"""
Created on Thu Jul 19 13:31:11 2018

@author: lbrevaul
"""

import numpy as np
import copy
from openmdao.api import ExplicitComponent
from scipy import interpolate, integrate

from trajectory.Simulation_stage_1 import simulation_stage_1
from trajectory.Simulation_stage_2 import simulation_stage_2

from trajectory.Event_meco import event_meco
from trajectory.Event_fairing import event_fairing_jettison
from trajectory.Event_command_stage_1 import event_command_stage_1_theta
from trajectory.Event_seco import event_seco,event_seco_mass
from trajectory.Event_impact import event_impact

import constants as Cst
import geopy.distance as dist
import specifications as Spec


class Trajectory_comp(ExplicitComponent):

    def setup(self):

        #Inputs
        self.add_input('Diameter_stage_1', val=4.6)
        self.add_input('Diameter_stage_2',val=4.6)
        self.add_input('Mass_flow_rate_stage_1', val=219.)
        self.add_input('Mass_flow_rate_stage_2', val=219.)
        self.add_input('N_eng_stage_1', val=7.)
        self.add_input('N_eng_stage_2', val=1.)
        self.add_input('OF_stage_1',val=3.)
        self.add_input('OF_stage_2',val=3.)
        self.add_input('Isp_stage_1',val=320.)
        self.add_input('Isp_stage_2',val=359.)
        self.add_input('Prop_mass_stage_1',val=350000.)
        self.add_input('Prop_mass_stage_2',val=75000.)
        self.add_input('Dry_mass_stage_1',val= 24000.)
        self.add_input('Dry_mass_stage_2',val=3994.)
        self.add_input('Pitch_over_duration',val=5.)
        self.add_input('thetacmd_i',val=2.72)
        self.add_input('thetacmd_f',val=20.)
        self.add_input('ksi',val=0.293)
        self.add_input('Exit_nozzle_area_stage_1',val = 0.82)
        self.add_input('Exit_nozzle_area_stage_2',val = 1.)
        self.add_input('Delta_vertical_phase',val=10.)
        self.add_input('Delta_theta_pitch_over',val = 1.)        
        self.add_input('Table_CX_complete_ascent' ,shape = (20,15))
        self.add_input('Mach_table',val =np.ones(20))  
        self.add_input('AoA_table',val =np.ones(15))  
        self.add_input('command_stage_1_exo',val =np.array([1.,1.]))  
        
        
        ## Outputs
        self.add_output('T_ascent',shape = 4000)   
        self.add_output('alt_ascent',shape = 4000)
        self.add_output('flux_ascent',shape = 4000)
        self.add_output('r_ascent',shape = 4000)
        self.add_output('V_ascent',shape = 4000)
        self.add_output('theta_ascent',shape = 4000)
        self.add_output('alpha_ascent',shape = 4000)
        self.add_output('nx_ascent',shape = 4000)
        self.add_output('alpha_cont',val= 10.)
        self.add_output('Nb_pt_ascent',val= 100.)
        self.add_output('m_ascent',shape = 4000)
        self.add_output('CX_ascent',shape = 4000)
        self.add_output('GLOW',val= 38000.)
        self.add_output('lat_ascent',shape = 4000)
        self.add_output('gamma_ascent',shape = 4000)
        self.add_output('longi_ascent',shape = 4000)
        self.add_output('thrust_ascent',shape = 4000)
        self.add_output('mass_flow_rate_ascent',shape = 4000)
        self.add_output('Mach_ascent',shape = 4000)
        self.add_output('pdyn_ascent',shape = 4000)
        self.add_output('rho_ascent',shape = 4000)
        self.add_output('distance_ascent',shape = 4000)
        self.add_output('state_separation_stage_1',shape=(5,)) 
        self.add_output('max_pdyn_load_ascent_stage_1',val=40e3) 

    def compute(self, inputs, outputs):
        
        #Integration parameters 
        atol_integration =1e-4
        rtol_integration=1e-4
        integration_method = 'BDF' #Implicit method based on backward-differentiation formulas see Scipy for more details
        step=2.

        #Constant definition
        Constants = {}
        Constants['r0'] = Cst.r0
        Constants['V0'] = Cst.V0
        Constants['gamma0'] = Cst.gamma0
        Constants['longi0'] = Spec.specifications['launch_site']['longitude']*np.pi/180. 

        Constants['longi_Kourou'] =Spec.specifications['launch_site']['longitude']
        Constants['lat_Kourou'] = Spec.specifications['launch_site']['latitude']
        Constants['Payload_mass'] = Spec.specifications['mission']['Payload_mass']
        Constants['Fairing_mass'] =Spec.specifications['stage_2_additional_masses']['Fairing_mass']

        Duration_stage_separation = Cst.Duration_stage_separation

        ######################## 1st stage definition ###################################
        #Mass and time definition
        tf1 = (inputs['Prop_mass_stage_1'])/(inputs['N_eng_stage_1']*inputs['Mass_flow_rate_stage_1']*0.8) #time of flight
        initial_mass = inputs['Dry_mass_stage_1'] + inputs['Dry_mass_stage_2'] +\
                            inputs['Prop_mass_stage_1'] + inputs['Prop_mass_stage_2']+\
                            Constants['Payload_mass'] + Constants['Fairing_mass'] #Gross Lift-Off Weight (GLOW)

        outputs['GLOW'] = initial_mass #Gross Lift-Off Weight (GLOW)
        final_mass_stage_1 = initial_mass -inputs['Prop_mass_stage_1'][0] #Supposed final mass after stage 1 propellant combustion
        table_alt_theta_stage_1 = np.array([-1.,2000e3])  #table for pitch angle interpolation as a function of altitude
        interp_theta_stage_1 = interpolate.interp1d(table_alt_theta_stage_1,inputs['command_stage_1_exo'],kind='linear') #interpolant for pitch angle control

        #Aerodynamics definition (table of Mach, Incidence (alpha), and resulting drag coefficient CX)
        Table_Mach_ = inputs['Mach_table'] 
        Table_incidence = inputs['AoA_table']
        Interp2_CX_complete_launcher=interpolate.interp2d(Table_incidence,Table_Mach_,inputs['Table_CX_complete_ascent'])
        
        ######################## 2nd stage definition ###################################
        tf2 = inputs['Prop_mass_stage_2']/(inputs['Mass_flow_rate_stage_2'][0]*(1-(1-Cst.derating_stage_2)))  #time of flight 2nd stage
        final_mass_stage_2 = inputs['Dry_mass_stage_2'] + Constants['Payload_mass']  #expected final mass of the 2nd stage

        ##################### Definition of parameter dictionary for 1st stage ###############
        param_integration_stage_1 = {}       
        param_integration_stage_1['aero'] = {}
        param_integration_stage_1['aero']['Interp_CX_stage_1'] = Interp2_CX_complete_launcher
        
        param_integration_stage_1['propulsion'] = {}
        param_integration_stage_1['propulsion']['Mass_flow_rate']=inputs['Mass_flow_rate_stage_1'][0]
        param_integration_stage_1['propulsion']['Isp']=inputs['Isp_stage_1'][0]  #specific impulse estimated by propulsion discipline
        param_integration_stage_1['propulsion']['N_eng']=inputs['N_eng_stage_1'][0] #number of engines
        
        param_integration_stage_1['masses'] = {}
        param_integration_stage_1['masses']['Mass_f']= final_mass_stage_1
        
        param_integration_stage_1['command'] = {}
        param_integration_stage_1['command']['Pitch_over_duration'] = inputs['Pitch_over_duration'][0] #duration of the pitch over manoeuver
        param_integration_stage_1['command']['Delta_theta_pitch_over'] = inputs['Delta_theta_pitch_over'][0] #angle of pitch for the pitch over manoeuver
        param_integration_stage_1['command']['Delta_vertical_phase'] = inputs['Delta_vertical_phase'][0] #duration of the vertical lift-off phase
        param_integration_stage_1['command']['Interp_theta_stage_1'] = interp_theta_stage_1
        
        param_integration_stage_1['geometry'] = {}
        param_integration_stage_1['geometry']['Exit_nozzle_area']=inputs['Exit_nozzle_area_stage_1'][0]
        param_integration_stage_1['geometry']['Diameter']=inputs['Diameter_stage_1'][0]
        
        param_integration_stage_1['simu'] = {}
        param_integration_stage_1['simu']['Mode_simu']= 1. #integration (1) or simulation (0)

       
        ############ structure for output data ###############
        data_simu = {}
        data_simu['T'] =  np.empty((1))     #Time (s)
        data_simu['nx'] = np.empty((1))     #Axial load factor
        data_simu['Mach'] = np.empty((1))   #Mach    
        data_simu['pdyn'] = np.empty((1))   #Dynamic pressure (W/m2)
        data_simu['flux'] = np.empty((1))   #Heat flux (kg m^-2 s^-1)   
        data_simu['alt'] = np.empty((1))    #altitude (m)   
        data_simu['alpha'] = np.empty((1))  #angle of attack (rad)      
        data_simu['theta'] = np.empty((1))  #pitch angle (rad)     
        data_simu['r'] = np.empty((1))      #altitude from the center of the Earth (m)      
        data_simu['V'] = np.empty((1))      #norm of velocity (m/s) 
        data_simu['gamma'] = np.empty((1))  #flight path angle (rad)
        data_simu['rho'] = np.empty((1))    #air density (kg/m3)   
        data_simu['CX'] = np.empty((1))     #drag coefficient  
        data_simu['thrust'] = np.empty((1)) #thrust (N)    
        data_simu['mass_flow_rate'] = np.empty((1))  #mass flow rate (kg/s)     
        data_simu['lat'] = np.empty((1))    #latitude (rad)  
        data_simu['longi'] = np.empty((1))  #longitude (rad)     
        data_simu['m'] = np.empty((1))      #launcher mass (kg)
        data_simu['distance'] = np.empty((1)) #distance from launch pad (m)      

           
        # definition of considered events
        event_fairing_ = lambda t,x :event_fairing_jettison(t,x,param_integration_stage_1) #event jettison fairing if heat flux below 1135 (kg m^-2 s^-1) 
        event_fairing_.terminal = True
        event_fairing_.direction = -1

        event_meco_ = lambda t,x :event_meco(t,x,param_integration_stage_1) #main engine cut-off if propellant mass = 0.
        event_meco_.terminal = True
        event_meco_.direction = -1     
        
        event_exo_flight_ = lambda t,x :event_command_stage_1_theta(t,x,param_integration_stage_1) #end of gravity turn
        event_exo_flight_.terminal = True
        event_exo_flight_.direction = -1       
                
        event_impact_ = lambda t,x :event_impact(t,x,param_integration_stage_1) #launcher impact the ground
        event_impact_.terminal = True
        event_impact_.direction = -1  
        
        fonction_ode_integration = lambda t,x :simulation_stage_1(t,x,param_integration_stage_1)  #launcher simulation equations of motion
        
        param_simu_stage_1 = copy.deepcopy(param_integration_stage_1)              
        param_simu_stage_1['simu']['Mode_simu']=0.
        
        fonction_ode_simu = lambda t,x :simulation_stage_1(t,x,param_simu_stage_1)  #launcher simulation equations of motion

        initial_state = np.array([Constants['r0'],\
                        Constants['V0'],\
                        Constants['gamma0'],\
                        Constants['longi0'],\
                        initial_mass]) #initial state of launcher
        
        span_integration = (0.,tf1)
        
        #settings of event caracteristics
        dico_events_stage_1 = {}
        dico_events_stage_1['MECO'] = {}
        dico_events_stage_1['MECO']['actif'] = False
        dico_events_stage_1['MECO']['instant'] = 0.
        dico_events_stage_1['MECO']['state'] = 0.
        
        dico_events_stage_1['fairing']  = {}
        dico_events_stage_1['fairing']['actif'] = False
        dico_events_stage_1['fairing']['instant'] = 0.
        dico_events_stage_1['fairing']['state'] = 0.

        dico_events_stage_1['exo_flight'] = {}
        dico_events_stage_1['exo_flight']['actif'] = False
        dico_events_stage_1['exo_flight']['instant'] = 0.  
        dico_events_stage_1['exo_flight']['state'] = 0.         
        
        dico_events_stage_1['impact'] = {}
        dico_events_stage_1['impact']['actif'] = False
        dico_events_stage_1['impact']['instant'] = 0.
        dico_events_stage_1['impact']['state']=  0.
        
        dico_events_stage_1['list_name_events'] = ['MECO','fairing','exo_flight','impact']
        dico_events_stage_1['list_events'] = [event_meco_,event_fairing_,event_exo_flight_,event_impact_]

        num_phase = 0
        final_time = 0.
        ##### integration of stage 1 trajectory
        while dico_events_stage_1['MECO']['actif'] == False and dico_events_stage_1['impact']['actif'] == False and final_time <tf1-0.1 :

            current_sol = integrate.solve_ivp(fonction_ode_integration,span_integration, initial_state,
                                      atol=atol_integration,rtol=rtol_integration,
                                      dense_output = True,method=integration_method,
                                      events =  dico_events_stage_1['list_events'])  #integration of equations of motion

            current_T = np.append(np.arange(current_sol.t[0],current_sol.t[-1],step),current_sol.t[-1])
            current_NX = np.zeros([len(current_T),1])
            current_Mach = np.zeros([len(current_T),1])
            current_Pdyn = np.zeros([len(current_T),1])
            current_flux = np.zeros([len(current_T),1])
            current_alt = np.zeros([len(current_T),1])
            current_alpha = np.zeros([len(current_T),1])
            current_gamma = np.zeros([len(current_T),1])
            current_theta = np.zeros([len(current_T),1])
            current_V = np.zeros([len(current_T),1])
            current_rho = np.zeros([len(current_T),1])
            current_CX = np.zeros([len(current_T),1])
            current_r = np.zeros([len(current_T),1])
            current_distance = np.zeros([len(current_T),1])
            current_thrust = np.zeros([len(current_T),1])
            current_lat = np.zeros([len(current_T),1])
            current_longi = np.zeros([len(current_T),1])
            current_m = np.zeros([len(current_T),1])
            current_mass_flow_rate = np.zeros([len(current_T),1])

            #Post traitment of the ODE integration to save the interesting data 
            for i in range(len(current_T)):
                (current_r[i], current_V[i], current_gamma[i], current_longi[i], current_m[i], current_NX[i],
                current_Mach[i],current_Pdyn[i],current_flux[i],current_alt[i],current_alpha[i],current_theta[i],
                current_rho[i], current_CX[i],current_thrust[i],current_mass_flow_rate[i],
                current_distance[i],current_lat[i]) = fonction_ode_simu(current_T[i],current_sol.sol(current_T[i]))
                        
            initial_state = current_sol.y[:,-1].copy()

            # checking for events and modification of state if needed
            for j in range(len(dico_events_stage_1['list_events'])):
                if len(current_sol.t_events[j])>0:
                    dico_events_stage_1[dico_events_stage_1['list_name_events'][j]]['actif'] = True 
                    dico_events_stage_1[dico_events_stage_1['list_name_events'][j]]['instant'] = current_sol.t_events[j][0] 
                    dico_events_stage_1[dico_events_stage_1['list_name_events'][j]]['state']=  current_sol.y[:,-1].copy()
                    
                    if dico_events_stage_1['list_name_events'][j] == 'fairing': ### largage coiffe             
                        initial_state[-1] = initial_state[-1] - Constants['Fairing_mass']
                        
            # update of the list of events
            for k in reversed(range(len(dico_events_stage_1['list_events']))):
                if len(current_sol.t_events[k])>0:
                    del dico_events_stage_1['list_events'][k]
                    del dico_events_stage_1['list_name_events'][k]
                   
            span_integration = (current_sol.t[-1],tf1)
            final_time = current_sol.t[-1]

            # save of trajectory data of the current phase 
            if num_phase == 0:
                data_simu['T'] =  np.array([current_T]).T
                data_simu['nx'] = current_NX
                data_simu['Mach'] = current_Mach    
                data_simu['pdyn'] = current_Pdyn     
                data_simu['flux'] = current_flux 
                data_simu['r'] = current_r      
                data_simu['alt'] = current_alt   
                data_simu['alpha'] = current_alpha*180/np.pi
                data_simu['gamma'] = current_gamma*180/np.pi
                data_simu['theta'] = current_theta*180/np.pi
                data_simu['V'] = current_V
                data_simu['rho'] = current_rho
                data_simu['CX'] = current_CX
                data_simu['thrust'] = current_thrust
                data_simu['mass_flow_rate'] = current_mass_flow_rate
                data_simu['lat'] = current_lat*180/np.pi
                data_simu['longi'] = current_longi*180/np.pi
                data_simu['m'] =current_m
                data_simu['distance'] =current_distance


            else:
                 data_simu['T'] = np.concatenate((data_simu['T'],np.array([current_T]).T))
                 data_simu['nx'] = np.concatenate((data_simu['nx'],current_NX))
                 data_simu['Mach'] = np.concatenate((data_simu['Mach'],current_Mach))
                 data_simu['pdyn'] = np.concatenate((data_simu['pdyn'],current_Pdyn))
                 data_simu['flux'] = np.concatenate((data_simu['flux'],current_flux))
                 data_simu['alt'] = np.concatenate((data_simu['alt'],current_alt))
                 data_simu['alpha'] = np.concatenate((data_simu['alpha'],current_alpha*180/np.pi))
                 data_simu['gamma'] = np.concatenate((data_simu['gamma'],current_gamma*180/np.pi))
                 data_simu['theta'] = np.concatenate((data_simu['theta'],current_theta*180/np.pi))
                 data_simu['V'] = np.concatenate((data_simu['V'],current_V))
                 data_simu['rho'] = np.concatenate((data_simu['rho'],current_rho))
                 data_simu['CX'] = np.concatenate((data_simu['CX'],current_CX))
                 data_simu['thrust'] = np.concatenate((data_simu['thrust'],current_thrust))
                 data_simu['mass_flow_rate'] = np.concatenate((data_simu['mass_flow_rate'],current_mass_flow_rate))
                 data_simu['lat'] = np.concatenate((data_simu['lat'],current_lat*180/np.pi))
                 data_simu['longi'] = np.concatenate((data_simu['longi'],current_longi*180/np.pi))
                 data_simu['m'] = np.concatenate((data_simu['m'],current_m))
                 data_simu['distance'] =np.concatenate((data_simu['distance'],current_distance))
                 data_simu['r'] =np.concatenate((data_simu['r'],current_r))

            num_phase = num_phase + 1
            
        state_separation_stage_1 = current_sol.y[:,-1].copy() 
        outputs['state_separation_stage_1'] = state_separation_stage_1
        outputs['max_pdyn_load_ascent_stage_1'] = np.max(data_simu['pdyn'])
        
####################################### 2nd stage ####################################################################

        instant_end_flight_stage_1 = current_sol.t[-1]
        outputs['alpha_cont']=np.max(np.abs(data_simu['alpha']))


        if dico_events_stage_1['impact']['actif'] == False:
            ################ definition of parameters for 2nd stage ###############
            param_integration_stage_2 = {}       
            param_integration_stage_2['aero'] = {}
            
            param_integration_stage_2['propulsion'] = {}
            param_integration_stage_2['propulsion']['Mass_flow_rate']=inputs['Mass_flow_rate_stage_2'][0]
            param_integration_stage_2['propulsion']['Isp']=inputs['Isp_stage_2'][0]
            param_integration_stage_2['propulsion']['N_eng']=inputs['N_eng_stage_2'][0]
                
            param_integration_stage_2['masses'] = {}
            param_integration_stage_2['masses']['Mass_f']=final_mass_stage_2
            
            param_integration_stage_2['command'] = {}
            param_integration_stage_2['command']['Theta_i']=inputs['thetacmd_i'][0]
            param_integration_stage_2['command']['Theta_f']=inputs['thetacmd_f'][0]
            param_integration_stage_2['command']['Ksi']=inputs['ksi'][0]
            
            param_integration_stage_2['geometry'] = {}
            param_integration_stage_2['geometry']['Exit_nozzle_area']=inputs['Exit_nozzle_area_stage_2'][0]
            param_integration_stage_2['geometry']['Diameter']=inputs['Diameter_stage_2'][0]
            
            param_integration_stage_2['simu'] = {}
            param_integration_stage_2['simu']['Mode_simu']= 1.#integration (1)ou simulation (0)
            param_integration_stage_2['simu']['Duration_separation'] = Duration_stage_separation
            param_integration_stage_2['simu']['Duration_flight'] = tf2
             
            initial_state_stage_2 = state_separation_stage_1.copy()
             
            ##### definition of events to be considered #####
    
            event_seco_ = lambda t,x :event_seco(t,x,param_integration_stage_2)  #Second stage engine cut-off
            event_seco_.terminal = True
            event_seco_.direction = 1
    #            
            event_fairing = lambda t,x :event_fairing_jettison(t,x,param_integration_stage_2) #Second stage fairing jettison
            event_fairing.terminal = True
            event_fairing.direction = -1 
            
            event_impact_ = lambda t,x :event_impact(t,x,param_integration_stage_2) ##Second stage Earth impact
            event_impact_.terminal = True
            event_impact_.direction = -1  
            
            fonction_ode_integration = lambda t,x :simulation_stage_2(t,x,param_integration_stage_2) #Simulation of the 2nd stage - equations of motion
            
            param_simu_stage_2 = copy.deepcopy(param_integration_stage_2)              
            param_simu_stage_2['simu']['Mode_simu']=0.
            
            fonction_ode_simu = lambda t,x :simulation_stage_2(t,x,param_simu_stage_2) #Simulation of the 2nd stage - equations of motion
            
            dico_events_stage_2 = {}
             
            dico_events_stage_2['SECO'] = {}
            dico_events_stage_2['SECO']['actif'] = False
            dico_events_stage_2['SECO']['instant'] = 0.
            dico_events_stage_2['SECO']['state']=  0.
            
            dico_events_stage_2['impact'] = {}
            dico_events_stage_2['impact']['actif'] = False
            dico_events_stage_2['impact']['instant'] = 0.
            dico_events_stage_2['impact']['state']=  0.
            
            if dico_events_stage_1['fairing']['actif'] == False:
                
                dico_events_stage_2['fairing']  = {}
                dico_events_stage_2['fairing']['actif'] = False
                dico_events_stage_2['fairing']['instant'] = 0.
                dico_events_stage_2['fairing']['state'] = 0.
    
                initial_state_stage_2[-1]= inputs['Dry_mass_stage_2'] + inputs['Prop_mass_stage_2']+Constants['Payload_mass'] + Constants['Fairing_mass']
    
                dico_events_stage_2['list_name_events'] = ['SECO','fairing','impact']
                dico_events_stage_2['list_events'] = [event_seco_,event_fairing,event_impact_]
            else :
                initial_state_stage_2[-1]= inputs['Dry_mass_stage_2'] + inputs['Prop_mass_stage_2']+Constants['Payload_mass']
                dico_events_stage_2['list_name_events'] = ['SECO','impact']
                dico_events_stage_2['list_events'] = [event_seco_,event_impact_]
    
            span_integration = (0.,tf2)
            current_T = np.zeros(1)
            ##### integration of stage 2 trajectory #####
            while (dico_events_stage_2['SECO']['actif'] == False and dico_events_stage_2['impact']['actif'] == False) and current_T[-1]<tf2-0.1 :
                
                current_sol = integrate.solve_ivp(fonction_ode_integration,span_integration, initial_state_stage_2,
                                          atol=atol_integration,rtol=rtol_integration,
                                          dense_output = True,method=integration_method,
                                          events =  dico_events_stage_2['list_events'])
    
                current_T = np.append(np.arange(current_sol.t[0],current_sol.t[-1],step),current_sol.t[-1])
                
                current_NX = np.zeros([len(current_T),1])
                current_Mach = np.zeros([len(current_T),1])
                current_Pdyn = np.zeros([len(current_T),1])
                current_flux = np.zeros([len(current_T),1])
                current_alt = np.zeros([len(current_T),1])
                current_alpha = np.zeros([len(current_T),1])
                current_gamma = np.zeros([len(current_T),1])
                current_theta = np.zeros([len(current_T),1])
                current_V = np.zeros([len(current_T),1])
                current_rho = np.zeros([len(current_T),1])
                current_CX = np.zeros([len(current_T),1])
                current_r = np.zeros([len(current_T),1])
                current_distance = np.zeros([len(current_T),1])
                current_thrust = np.zeros([len(current_T),1])
                current_lat = np.zeros([len(current_T),1])
                current_longi = np.zeros([len(current_T),1])
                current_m = np.zeros([len(current_T),1])
                current_mass_flow_rate = np.zeros([len(current_T),1])
                
                #Post traitment of the 2nd stage ODE integration to save interesting data
                for i in range(len(current_T)):
                    (current_r[i], current_V[i], current_gamma[i], current_longi[i], current_m[i], current_NX[i],
                    current_Mach[i],current_Pdyn[i],current_flux[i],current_alt[i],current_alpha[i],current_theta[i],
                    current_rho[i], current_CX[i],current_thrust[i],current_mass_flow_rate[i],
                    current_distance[i],current_lat[i]) = fonction_ode_simu(current_T[i],current_sol.sol(current_T[i]))
                
                initial_state = current_sol.y[:,-1].copy()
    
                #### checking for events and modification of state if needed ####
                for j in range(len(dico_events_stage_2['list_events'])):
                    if len(current_sol.t_events[j])>0:
                        dico_events_stage_2[dico_events_stage_2['list_name_events'][j]]['actif'] = True 
                        dico_events_stage_2[dico_events_stage_2['list_name_events'][j]]['instant'] = current_sol.t_events[j][0]+instant_end_flight_stage_1
                        dico_events_stage_2[dico_events_stage_2['list_name_events'][j]]['state'] = current_sol.y[:,-1].copy()
    
                        if dico_events_stage_2['list_name_events'][j] == 'fairing':             
                            initial_state[-1] = initial_state[-1] - Constants['Fairing_mass']
                
                #update of list of events
                for k in reversed(range(len(dico_events_stage_2['list_events']))):
                    if len(current_sol.t_events[k])>0:
                        del dico_events_stage_2['list_events'][k]
                        del dico_events_stage_2['list_name_events'][k]
                       
                span_integration = (current_sol.t[-1],tf2)
    
                #save of trajectory data 
                data_simu['T'] = np.concatenate((data_simu['T'],instant_end_flight_stage_1+np.array([current_T]).T))
                data_simu['nx'] = np.concatenate((data_simu['nx'],current_NX))
                data_simu['Mach'] = np.concatenate((data_simu['Mach'],current_Mach))
                data_simu['pdyn'] = np.concatenate((data_simu['pdyn'],current_Pdyn))
                data_simu['flux'] = np.concatenate((data_simu['flux'],current_flux))
                data_simu['alt'] = np.concatenate((data_simu['alt'],current_alt))
                data_simu['alpha'] = np.concatenate((data_simu['alpha'],current_alpha*180/np.pi))
                data_simu['gamma'] = np.concatenate((data_simu['gamma'],current_gamma*180/np.pi))
                data_simu['theta'] = np.concatenate((data_simu['theta'],current_theta*180/np.pi))
                data_simu['V'] = np.concatenate((data_simu['V'],current_V))
                data_simu['rho'] = np.concatenate((data_simu['rho'],current_rho))
                data_simu['CX'] = np.concatenate((data_simu['CX'],current_CX))
                data_simu['thrust'] = np.concatenate((data_simu['thrust'],current_thrust))
                data_simu['mass_flow_rate'] = np.concatenate((data_simu['mass_flow_rate'],current_mass_flow_rate))
                data_simu['lat'] = np.concatenate((data_simu['lat'],current_lat*180/np.pi))
                data_simu['longi'] = np.concatenate((data_simu['longi'],current_longi*180/np.pi))
                data_simu['m'] = np.concatenate((data_simu['m'],current_m))
                data_simu['distance'] =np.concatenate((data_simu['distance'],current_distance))
                data_simu['r'] =np.concatenate((data_simu['r'],current_r))
 
            
        #### Outputs data for OpenMDAO
        Nb_pt = len(data_simu['T'])
        outputs['Nb_pt_ascent'] = Nb_pt
        outputs['T_ascent'][0:Nb_pt] = data_simu['T'].T[0]
        outputs['r_ascent'][0:Nb_pt] = data_simu['r'].T[0]
        outputs['nx_ascent'][0:Nb_pt] = data_simu['nx'].T[0]
        outputs['Mach_ascent'][0:Nb_pt] = data_simu['Mach'].T[0]     
        outputs['pdyn_ascent'][0:Nb_pt] = data_simu['pdyn'].T[0]     
        outputs['flux_ascent'][0:Nb_pt] = data_simu['flux'].T[0]     
        outputs['alt_ascent'][0:Nb_pt] = data_simu['alt'].T[0]    
        outputs['alpha_ascent'][0:Nb_pt] = data_simu['alpha'].T[0]
        outputs['gamma_ascent'][0:Nb_pt] = data_simu['gamma'].T[0]
        outputs['theta_ascent'][0:Nb_pt] = data_simu['theta'].T[0]
        outputs['V_ascent'][0:Nb_pt] = data_simu['V'].T[0]  
        outputs['rho_ascent'][0:Nb_pt] = data_simu['rho'].T[0]   
        outputs['CX_ascent'][0:Nb_pt] = data_simu['CX'].T[0]       
        outputs['thrust_ascent'][0:Nb_pt] = data_simu['thrust'].T[0]
        outputs['mass_flow_rate_ascent'][0:Nb_pt] = data_simu['mass_flow_rate'].T[0]       
        outputs['lat_ascent'][0:Nb_pt] = data_simu['lat'].T[0]    
        outputs['longi_ascent'][0:Nb_pt] = data_simu['longi'].T[0]   
        outputs['m_ascent'][0:Nb_pt] = data_simu['m'].T[0]  
        outputs['distance_ascent'][0:Nb_pt] = data_simu['distance'].T[0]     
        
        #Definition of additional data for post-treatment
        self.data_events={}
        self.data_events['stage_1'] = dico_events_stage_1
        if dico_events_stage_1['impact']['actif'] == False:
            self.data_events['stage_2'] = dico_events_stage_2
        
        self.data_simu={}
        self.data_simu['ascent'] = data_simu
        

        
