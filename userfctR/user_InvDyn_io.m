function [] = user_InvDyn_io(mbs_data,tsim,step,flag)               
% --------------------------               
% UCL-CEREM-MBS               
%               
% @version MBsysLab_m 1.7.a               
%               
% Creation : 2005               
% Last update : 30/09/2008               
% -------------------------               
%               
% user_InvDyn_io(mbs_data,tsim,step,flag)               
%               
% mbs_data : multibody data structure               
% tsim : current step time               
% step :               
%   - before the process :                
%           . flag = 'init'               
%           . step contains the exact number of requested time steps               
%   - during the process :                
%           . flag : non-existent                
%           . step : contains the current process step (1,2,3,...)               
%               
% no return value               
% this function may use a global structure called MBS_user               
               
% MBS_user : global user structure               
%               
% field "curvar" : to compute and store in any user function               
%                  the current value of a variable               
%                  ex. "curvar.myangle"               
% field "resinvdyn" : to store the corresponding variable at each step "step"               
%                     corresponding to the step time "tsim".               
               
global MBS_user MBS_info               
               
if (nargin > 3)         %   process starting               
    switch flag               
        case 'init'                    
            MBS_user.resinvdyn.tsim = zeros(step,1);               
%             MBS_user.resinvdyn.myangle = zeros(step,1);               
               
                       
            %extForces storage for animation               
            for ixF=1:mbs_data.Nxfrc               
                MBS_user.resdirdyn.extForces(ixF).P=zeros(step,3);               
                MBS_user.resdirdyn.extForces(ixF).R=zeros(step,4);               
                MBS_user.resdirdyn.extForces(ixF).F=zeros(step,3);               
            end               
                       
        otherwise       %   unused               
            ;               
    end               
else                    %   process running               
    MBS_user.resinvdyn.tsim(step) = tsim;               
%     MBS_user.resinvdyn.myangle(step) = MBS_user.curvar.myangle;               
               
    %extForces storage for animation               
    if isfield(MBS_user,'curvar')                     
        if isfield(MBS_user.curvar,'extForces')               
            for ixF=1:mbs_data.Nxfrc               
                MBS_user.resdirdyn.extForces(ixF).P(step,:)=MBS_user.curvar.extForces(ixF).P;               
                MBS_user.resdirdyn.extForces(ixF).R(step,:)=mbs_Rot2vrml(MBS_user.curvar.extForces(ixF).R);               
                MBS_user.resdirdyn.extForces(ixF).F(step,:)=MBS_user.curvar.extForces(ixF).F;               
            end               
        end               
    end                                               
end               
               
return               
