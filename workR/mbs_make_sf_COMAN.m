% -------------------------------------------------------------------------   
% Allan Barrea and Nicolas Van der Noot - November 2012   
% Compiles the C files to a MEX-file for the CoMan project   
% Based on 'mbs_make_sf.m' (c) 2008 CEREM, UCL   
%   
% This file should be executed on the workR folder of the project !   
% -------------------------------------------------------------------------   
   
function mbs_make_sf_COMAN  
   
% % --- Initialization ---   
close all; clear variables; clc;                                            % Cleaning of the Matlab workspace   
global MBS_user;                                                            % Declaration of the global user structure   
MBS_user.process = '';                                                      % Initialisation of the user field "process"   
   
% Project loading   
prjname = 'COMAN_2D_LinkControl';   
[mbs_data, mbs_info] = mbs_load(prjname,'default');                         % Option 'default': automatic loading of "$project_name$.mbs"    
mbs_data_ini = mbs_data;                                                    % Backup of the initial multibody data structure   
   
mbs_data.DonePart=1;   
MBS_data.DonePart=1;   
%% getting initial conditions:   
% mbs_data.q=1;   
mbs_sf_create_project(mbs_info,1,'dirdynared',0);   
   
return   
clc;   
clear functions; % to free the MEX-file if needed   
   
tmp_obj_dir_created = 0;   
   
if(strcmp(computer, 'PCWIN'))   
    disp('MBS>> Running MEX-file creation on Windows...');   
elseif(strcmp(computer, 'MACI64'))   
    disp('MBS>> Running MEX-file creation on MacOS X 64 bits...');   
else   
    disp('MBS>> ERROR - Unsupported OS !!');   
    return;   
end   
   
% MBSPATHDEF defines the paths for use with MBsysLab routines   
%   mbsprjpath : Path to the directory containing your multibody systems projects   
%   mbspath    : Path to the directory containing MBsysLab files and directories   
mbspathdef;   
   
define = 'DIRDYNARED';   
prjname = 'CoMan_LegsCad_23Dof_FB_Ver1';   
   
% --- To avoid overwritting a correct file ---   
   
fname = strcat('mbs_sf_dirdynared_', prjname);   
   
if exist(fname,'file') == 3 % if there is already a 'fname' MEX-file   
    button = questdlg(['Would you like to replace ' fname ' ?'],'File already exist');   
    switch button   
        case {'No', 'Cancel'}   
            disp(['MBS>> SFunction file: ' fname ' not created']);   
            return   
    end   
end   
   
% --- Files and directories definitions ---   
   
common_dir = fullfile(mbspath,'MBsysLab','mbs_simulink','mbs_sourceC');   
user_dir = fullfile(mbsprjpath,prjname,'SfunctionsR','src_user');   
symbolic_dir = fullfile(mbsprjpath,prjname,'symbolicR');   
   
project_files = {...   
    'LocalDataStruct.c'...   
    'mbs_close_loops.c'...   
    'mbs_compute_model.c'...   
    'mbs_dirdynared.c'...   
    'mbs_sf_main.c'...   
    'MBSdataStruct.c'...   
    'MBSsensorStruct.c'...   
    'sf_InitCond.c'...   
    'sf_IOPort.c'...   
    };   
   
symbolic_files = {...   
    ['mbs_cons_hJ_' prjname '.c']...   
    ['mbs_cons_jdqd_' prjname '.c']...   
    ['mbs_dirdyna_' prjname '.c']...   
    ['mbs_extforces_' prjname '.c']...   
    ['mbs_gensensor_' prjname '.c']...   
    ['mbs_link_' prjname '.c']...   
    ['mbs_sensor_' prjname '.c']...   
    };   
   
tool_files = {...   
    'choldc.c'...   
    'cholsl.c'...   
    'lubksb.c'...   
    'ludcmp.c'...   
    'lut.c'...   
    'mbs_bakker.c'...   
    'mbs_calspan.c'...   
    'mbs_kine_wheel.c'...   
    'mbs_matrix.c'...   
    'mbs_tool.c'...   
    'norm.c'...   
    'nrutil.c'...   
    'svbksb.c'...   
    'svdcmp.c'...   
    };   
   
user_files = {...   
    'user_compute_output.c'...   
    'user_cons_hJ.c'...   
    'user_cons_jdqd.c'...   
    'user_Derivative.c'...   
    'user_DrivenJoints.c'...   
    'user_ExtForces.c'...   
    'user_GroundLevel.c'...   
    'user_initialization.c'...   
    'user_JointForces.c'...   
    'user_Link3Dforces.c'...   
    'user_Linkforces.c'...   
    'user_sf_IO.c'...   
    'UserModelStruct.c'...   
    };   
       
if ~exist('tmp_obj','dir'),   
    tmp_obj_dir_created = 1;   
    mkdir('tmp_obj');   
end   
   
% --- Compiling ---   
for i=1:length(project_files),   
    disp(['MBS>> Compiling ' project_files{i} '...']);   
    eval(['mex -c -D' define ' -I''' common_dir ''' -I''' user_dir ''' -outdir ''tmp_obj'' ''' fullfile(common_dir,project_files{i}) '''']);   
end   
   
for i=1:length(symbolic_files),   
    disp(['MBS>> Compiling ' symbolic_files{i} '...']);   
    eval(['mex -c -D' define ' -I''' symbolic_dir ''' -I''' common_dir ''' -I''' user_dir ''' -outdir ''tmp_obj'' ''' fullfile(symbolic_dir,symbolic_files{i}) '''']);   
end   
   
for i=1:length(tool_files),   
    disp(['MBS>> Compiling ' tool_files{i} '...']);   
    eval(['mex -c -D' define ' -I''' symbolic_dir ''' -I''' common_dir ''' -I''' user_dir ''' -outdir ''tmp_obj'' ''' fullfile(common_dir,tool_files{i}) '''']);   
end   
   
for i=1:length(user_files),   
    disp(['MBS>> Compiling ' user_files{i} '...']);   
    eval(['mex -c -D' define ' -I''' symbolic_dir ''' -I''' common_dir ''' -I''' user_dir ''' -outdir ''tmp_obj'' ''' fullfile(user_dir,user_files{i}) '''']);   
end   
   
clc;   
   
% --- Linking ---   
disp('MBS>> Linking dirdynared...');   
   
if(strcmp(computer, 'PCWIN'))   
    % Linker command for Windows   
    disp('MBS>> Linking dirdynared...');   
    eval(['mex -D' define '  -output ''mbs_sf_dirdynared_' prjname ''' tmp_obj/*.obj']);   
elseif(strcmp(computer, 'MACI64'))   
    % Adapting the linker command to MacOS X 64 bits   
    % Beware that the object files have a '.o' extension instead of '.obj'   
    % Beware that the 'tmp_obj/*.o' expression doesn't work on Mac   
    % so, all the file names have to be given explicitely.   
    objfiles = dir('tmp_obj/*.o');   
    objfiles = {objfiles.name};   
    objfiles = sprintf('tmp_obj/%s ', objfiles{:});   
    eval(['mex -largeArrayDims -D' define '  -output ''mbs_sf_dirdynared_' prjname ''' ' objfiles]);   
end   
   
% --- Cleaning ---   
% removes the 'tmp_obj' directory only if it was created by this function   
if(tmp_obj_dir_created)   
    disp('MBS>> Cleaning ''tmp_obj'' directory...');   
    delete('tmp_obj/*.*')   
    rmdir('tmp_obj');   
end   
   
% --- Ending message ---   
disp(['MBS>> SFunction file: ' fname ' successfully created']);   
   
end   
