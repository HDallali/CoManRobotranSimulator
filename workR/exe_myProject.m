%--------------------------------------------------------------------------  
%   Universit�catholique de Louvain  
%   CEREM : Centre for research in mechatronics  
%   http://www.robotran.be    
%   Contact : robotran@prm.ucl.ac.be  
%   Version : ROBOTRAN $Version$  
%  
%   MBsysLab main script template:  
%      - featuring default options  
%      - to be adapted by the user  
%  
%   Project : Pendulum Spring  
%   Author :  Nicolas Docquier  
%   Date :    07/10/2011  
%--------------------------------------------------------------------------  
  
%% 1. Initialization and Project Loading [mbs_load]  
%--------------------------------------------------------------------------  
close all; clear variables; clc;                                            % Cleaning of the Matlab workspace  
global MBS_user;                                                            % Declaration of the global user structure  
MBS_user.process = '';                                                      % Initialisation of the user field "process"  
  
% Project loading  
prjname = 'CoMan_Legs_7DofArm';  
[mbs_data, mbs_info] = mbs_load(prjname,'default');                         % Option 'default': automatic loading of "$project_name$.mbs"   
mbs_data_ini = mbs_data;                                                    % Backup of the initial multibody data structure  
 
mbs_sf_create_project(mbs_info,1,'dirdynared',1); 
 
return  
%% 3. direct dynamics [mbs_exe_dirdyn]  
%--------------------------------------------------------------------------  
MBS_user.process = 'dirdyn';  
  
% change inital conditions  
mbs_data.q(1) = pi/3;  
mbs_data.q(2) = 0.1;  
  
% set options  
opt.dirdyn = {'time',0:0.01:10,'motion','simulation',...  
    'odemethod','ode45','save2file','yes','framerate',1000,...  
    'renamefile','no','verbose','yes'};  
% other options : 'visualize', 'save2file', 'depinteg', 'dtmax', 'dtinit',  
%                 'reltol', 'abstol', 'clearmbsglobal'                      % Help about options on www.robotran.be  
  
% launch simulation  
[mbs_dirdyn,mbs_data] = mbs_exe_dirdyn(mbs_data,opt.dirdyn);                % Direct dynamics process (time simulation)  
  
%% 4. Graphical Results  
%--------------------------------------------------------------------------  
figure  
subplot(2,1,1)  
plot(mbs_dirdyn.tsim,mbs_dirdyn.q(:,1)/pi*180);                       
grid;  
xlabel('Time [s]'); ylabel(['Pendulum angle [�]']);  
subplot(2,1,2)  
plot(mbs_dirdyn.tsim,mbs_dirdyn.q(:,2));                       
grid;  
xlabel('Time [s]'); ylabel(['Slider displacement [m]']);  
