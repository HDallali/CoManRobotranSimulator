function [] = user_DirDyn_io(mbs_data,tsim,step,flag)               
% --------------------------               
% UCL-CEREM-MBS               
%               
% @version MBsysLab_m 1.7.a               
%               
% Creation : 2005               
% Last update : 30/09/2008               
% -------------------------               
%               
% user_DirDyn_io(mbs_data,tsim,step,flag)               
%               
% mbs_data : multibody data structure               
% tsim : current time step               
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
% field "resdirdyn" : to store the corresponding variable at each step "step"               
%                     corresponding to the step time "tsim".               
               
global MBS_user MBS_info               
               
if (nargin > 3)         %   process starting               
    switch flag               
        case 'init'               
            MBS_user.resdirdyn.tsim = zeros(step,1);               
%             MBS_user.resdirdyn.myangle = zeros(step,1);               
               
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
    MBS_user.resdirdyn.tsim(step) = tsim;               
%     MBS_user.resdirdyn.myangle(step) = MBS_user.curvar.myangle;               
    MBS_user.resdirdyn.GRF(:,:,step) = MBS_user.GRF;   
       
   
% --------------------------------------------------------------------       
% --------------------------------------------------------------------       
% Do the control calc. here.   
% Send the results with MBS_user....   
% --------------------------------------------------------------------       
% --------------------------------------------------------------------       
r=step;   
% A=MBS_user.PositionRefrences;   
VT= MBS_user.VTgdrives;   
  
q=mbs_data.q(7:29);   
qd=mbs_data.qd(7:29);   
 
qm=mbs_data.ux(1:23);   
qmd=mbs_data.ux(24:2*23);   
  
 
Traj = MBS_user.PositionRefrences; 
 
% r=find(Traj(:,1)>=tsim,1); 
if (r>length(Traj)) 
    r=length(Traj) 
end 
 
qref=Traj(r,2:end); 
 
% qref(22)=-0.20;  
% qref(24)= 0.45;  
% qref(25)=-1.60;  
%   
% qref(26)=-0.20;  
% qref(28)= -0.45;  
% qref(29)=-1.60;  
  
u=zeros(23,1);  
 
Kp=MBS_user.Control.Kp; 
Kd=MBS_user.Control.Kd; 
 
for i=7:29       
    u(i-6) =  -Kp*(qm(i-6) - qref(i))- Kd*qmd(i-6)  ;               
end   
               
%     converting voltage to torque:   
    u = VT*u;   
%    update the voltage in (MBS_user.u)   
MBS_user.u=u;    
      
MBS_user.resdirdyn.u(:,step) = MBS_user.u;   
MBS_user.resdirdyn.qref(:,step)=qref'; 
 
% --------------------------------------------------------------------       
       
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
