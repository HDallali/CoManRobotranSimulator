function [Qq] = user_JointForces(mbs_data,tsim);               
% --------------------------               
% UCL-CEREM-MBS               
%               
% @version MBsysLab_m 1.7.a               
%               
% Creation : 2006               
% Last update : 30/09/2008               
% -------------------------               
%               
%[Qq] = user_JointForces(mbs_data,tsim);               
%               
% mbs_data : multibody data structure               
% tsim : current time               
%               
% Qq : joint generalized force/torque (for all joints)               
% Qq(i) : joint force/torque in joint (i) along its joint axis               
%               
% this function may use a global structure called MBS_user               
               
global MBS_user MBS_info               
     
Qq = mbs_data.Qq;    
  
q=mbs_data.q(7:(23+6));    
qd=mbs_data.qd(7:(23+6));    
qm=mbs_data.ux(1:23);    
qmd=mbs_data.ux(24:2*23);    
 
KKs=MBS_user.KKs;    
DDs=MBS_user.DDs;    
Qq = KKs*(qm-q)+DDs*(qmd-qd);    
 
% adding viscose friction:           
%         C=20*eye(23);   
%         Qq=Qq-C*qd;   
% returning the final toruqes:   
 
k=100; 
 
Qq=[zeros(4,1);-k*mbs_data.q(5:6); Qq ];    
 
    
%/*-- End of user code --*/               
               
return         
   
