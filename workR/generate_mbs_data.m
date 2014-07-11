function [ mbs_data ] = generate_mbs_data()

% Generates the mbs_data structure 
% used in 'call_simulink' and in the xml file generation

%% Project loading

global MBS_user;       % Declaration of the global user structure
MBS_user.process = ''; 

prjname = 'CoMan_Legs_7DofArm';      % project name
[mbs_data, mbs_info] = mbs_load(prjname,'default');

% number of constraints 
% -> if there is no constraint, you must comment these lines 
%     ( 'mbs_data.Nuserc = 0;' might not work !!!)
mbs_data.Nuserc = 0;
mbs_data.Ncons  = 0;


end

