# -*- coding: utf-8 -*-
"""
Created on Thu Jul 19 13:31:11 2018

@author: lbrevaul
"""

import numpy as np
		
def sizing(Mprop, OF, D,type_fuel):
	#Calcul des volumes
	M_F = Mprop /(1+OF)
	M_OX = Mprop - M_F
	mu_LOX = 1141.0  
	if type_fuel == 'LH2':
		mu_F = 70.85
	elif type_fuel =='CH4':
		mu_F = 422.36
	elif type_fuel == 'RP1':
		mu_F = 810.0
	
	V_OX = M_OX / mu_LOX
	V_F = M_F / mu_F
	V_tot= V_F+V_OX
	h_dome = 0.3*D
	
	volume_domes = 4.0/3.0 * np.pi * (D/2)**2*h_dome
	p = 1.6075
	S_dome = (4.0*np.pi* (( (D/2.0)**(2.0*p) + 2* (D/2.0)**(p) * h_dome**p )/3.0)**(1./p))
	volume_virole_OX = V_OX - volume_domes
	L_virole_OX = volume_virole_OX / np.pi /(D/2)**2
	S_OX = L_virole_OX *(D/2)**2 * np.pi + S_dome
	volume_virole_F = V_F - volume_domes
	L_virole_F = volume_virole_F / np.pi /(D/2)**2
	S_F = L_virole_F *(D/2.)**2 * np.pi + S_dome    
	S_totale = S_OX+S_F
	S_exterieur = 2* np.pi*(L_virole_OX+L_virole_F+4*h_dome+0.5)*(D/2.)    
	L_total = L_virole_OX + L_virole_F + 4*h_dome + 0.5
	return S_OX, S_F, S_totale, S_dome, S_exterieur, L_total
	
def engine_mass(T, type_prop, feed): 

#T in N !!!
 
	if type_prop == 'Cryogenic':

		if (feed == 'SC'):
			a= -1.17899e08; 
			b = -7.380845e-01;
			c = +6.09805e03;

		elif (feed == 'EC'):
			a = -9.76421e4; 
			b = -4.27622e-1;
			c = +8.97980e2;

		elif (feed == 'GG'):
			a = 7.54354e-03; 
			b = 8.85635e-01;
			c = 2.02881e01;

		return a*(T)**b+c

	elif type_prop == 'Cryostorable':
		
		if (feed == 'SC'):
			if T<2050e3:
				a = 8.51852e-03; 
				b = 8.52826e-01;
				c = 1.06632e02;
			else:
				a = 1.65368e00; 
				b = 5.69842e-01;
				c = -4.37849e03;
			return a*(T)**b+c

		elif (feed == 'PF'):
			a = -2.13325e-09; 
			b = 1.7087e-03;
			c = 6.38629e00;
			return a*(T)**2+ b*T+c

		elif (feed == 'GG'):
			a = 3.75407e03; 
			b = 7.05627e-02;
			c = -8.8479e03;
			return a*(T)**b+c
		
		
		elif type_prop == 'Storable':
		
			if (feed == 'SC'):
				a= 4.74445e-01; 
				b = 5.35755e-01;
				c =-7.73681e00;
				return a*(T)**b+c

			elif (feed == 'GG'):
				a = 6.37913e00; 
				b = 3.53665e-01;
				c = -1.48832e02;
				return a*(T)**b+c

			elif (feed == 'PF'):
				a = -3.36532e-08; 
				b = 4.74402e-03;
				c = -1.93920e01;
				return a*(T)**2 + b*T+c	
			
def thrust_frame_mass(T,M_eng,n_ax_max,N_eng,material,SSM):

	#Parametres
	if material == 'Al':
		k_SM=1
	elif material == 'Composite':
		k_SM = 0.62
		
	#SSM = 2  # Structural Safety Margin
	return (0.013*N_eng**0.795*(224.81*T)**0.579+0.01*N_eng*(M_eng/(0.45))**0.717)*0.45*(1.5*SSM*n_ax_max*9.80665)*k_SM
	
def tank_mass(P_dyn_max,n_ax_max,P_tanks_Ox,P_tanks_F,V_FT,V_Ox,D,S_Ox,S_F,S_dome,S_totale,type_prop,config,type_stage,type_struct):

	#Mass tanks
	# Parametres
	k1=1.15  #Material
	S_tot = S_totale
	if config =='common_bulkhead':
		k2 = (S_tot - 1.5*S_dome)/S_tot  
	elif config == 'intertank':
		k2 = 1.

	k3=1.3 #Vertical integration 
	k4_ref = 5.76404
	k4=P_dyn_max**0.16/k4_ref
	SSM=1.25  #Structural #Safety Margin
	k_5_ref =  1.29134
	k5=(SSM*n_ax_max)**0.15/k_5_ref
	k6_ref = 2.7862
	k6_Ox=1.3012+1.4359*10**-6*P_tanks_Ox/k6_ref
	k6_F=1.3012+1.4359*10**-6*P_tanks_F/k6_ref
	### Mass Fuel tank
	M_FT=np.prod(np.array([k1,k2,k3,k4,k5,k6_F]))*((V_FT*35.315)*0.4856+800)*0.4536
	### Mass Ox tank

	M_OxT = np.prod(np.array([k1,k2,k3,k4,k5,k6_Ox]))*((V_Ox*35.315)*0.4856+700)*0.4536
	### Mass thermal protection
	k_ins_Ox = 0.9765
	k_ins_F = 1.2695

	M_TPS_OxT=k_ins_Ox*S_Ox
	
	if type_prop == 'Cryogenic':
		M_TPS_FT = k_ins_F*S_F
	else: 
		M_TPS_FT = 0
		
	if config == 'common_bulkhead':
		M_inter_tank = 0
	elif config == 'intertank':
		if type_stage =='lower':
			k_1 = 5.4015
			k_2 = 0.5169
		elif np.logical_or(type_stage == 'upper',type_stage == 'booster'):
			k_1 = 3.8664
			k_2 = 0.6025
			
		k_it = 0.3
		l_it = 2*k_it * D+0.5
		if type_struct == 'Al':
			k_SM = 1.
		elif type_struct =='Composite':
			k_SM = 0.8
			
		M_inter_tank = k_SM*k_1*D*np.pi*l_it*(D*3.2808)**k_2
		
	return M_FT,M_OxT,M_TPS_OxT,M_TPS_FT,M_inter_tank
	
def TVC_mass(T, techno):
	if techno == 'electromechanic':
		return 0.1078*(T*10**-3)+43.702
	elif techno == 'hydraulic':
		return 0.1976*(T*10**-3)+20.922
		
		
def EPS_avio_mass(S_tot,RL):

	#no rendundancy 
	
	#RL = 1
	#critical componenets redundancy 
	#RL = 2 
	#full redundancy
	#RL = 3
	if RL == 1: 
		K_RL = 0.7
	elif RL == 2:
		K_RL = 1.0
	elif RL ==3:
		K_RL = 1.3
	
	M_avio = K_RL*(246.76+1.3183*S_tot)


	M_EPS = K_RL*0.405*M_avio
	return M_avio, M_EPS
	
def mass_interstage(S,D_inf,D_up,U_L):

	#Coeff surete
	k_SM=1

	k_1_1=7.7165
	k_1_2=5.5234

	k_2_1=0.4856
	k_2_2=0.5210

	D=(D_inf+D_up)/2

	if U_L== 'lower':  # Lower stage
		return k_SM*k_1_1*S*(D*3.2808)**k_2_1 
	

	if U_L=='upper': # Upper stage
		 return k_SM*k_1_2*S*(D*3.2808)**k_2_2  