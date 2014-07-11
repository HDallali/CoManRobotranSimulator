% INITIALIZING ROBOTRAN SIMULATION    
% Continuous time, compliant model simulation of coman 
% - -------------------------------------------------- 
clc;clear all;    
global MBS_user;    
 
% Project loading    
prjname = 'CoMan_Legs_7DofArm';    
[mbs_data, mbs_info] = mbs_load(prjname,'default');      
mbs_data_ini = mbs_data;   
 

Num_sensor_MidWaist = mbs_get_S_sensor_id(mbs_info,'MidWaist');  
 
load MBS_user_data_compliant 
%% Creating a position trajectory 
t_max=15; % maximum simulation time 
 
disp('Please specify the type of motion you want to simulate, push up, squat, ... ' ) 
% f=input('please input the frequency of the desired motion between 0.1 & 2 hz '); 
% A=input('please input the amplitude of the desired motion between 0.1 & 1.0 rad '); 
f=0,A=0; 
 
if  isempty(f) 
    f=1; 
end 
 
if  isempty(A) 
    A=0.3; 
end 
 
type='pushup'; 
 
type='squat'; 
 
% type = 'armsup'; 
 
type = 'walk'; 
 
MBS_user.type=type; 
 
initialization; 
 
% push up trajectory 
Traj = Get_Trajectory(A, f, t_max, type, MBS_data); 
Traj=Traj(1000:end,:); 
 
 
%% solving dirdyn ODE, with events  
% set the initial conditions: 
set_ic2; 
 
run_sim; 
 
%% Plotting joint references:  
% myplots 
