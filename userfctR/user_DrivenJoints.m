function [q,qd,qdd] = user_DrivenJoints(mbs_data,tsim)               
% --------------------------               
% UCL-CEREM-MBS               
%               
% @version MBsysLab_m 1.7.a               
%               
% Creation : 2005               
% Last update : 30/09/2008               
% -------------------------               
%               
%[q,qd,qdd] = user_DrivenJoints(mbs_data,tsim)               
%               
% mbs_data : multibody data structure               
% tsim : current time               
%               
% q, qd, qdd : updated column vectors of generalized coordinates               
%               
%               
% mbs_data.q : generalized coordinates [column vector]               
% mbs_data.qd : generalized velocities [column vector]               
% mbs_data.qdd : generalized accelerations [column vector]               
% mbs_data.nqc : number of driven variables               
% mbs_data.qc : indices of driven variables [column vector]               
               
global MBS_user MBS_info               
               
 
%/*-- Begin of user code --*/               
   
q   = mbs_data.q;               
qd  = mbs_data.qd;               
qdd = mbs_data.qdd;  
 
 
%   for j=1:(mbs_data.nqc) 
%       i=mbs_data.qc(j); 
%       q(i) = 0.0;  
%       qd(i) = 0.0;  
%       qdd(i) = 0.0;  
% % % for air walking i use the: 
% %       if i==3 
% %           q(i)=1.0; 
% %       end 
%   end 
   
   
%     qdd(i) = f''(tsim) ...    
     
%     qd(i) = f'(tsim) ...               
%     qdd(i) = f''(tsim) ...               
    
%To eliminate the floating base dofs when linearizing    
%set floating base joints as driven, velocities and accelerations to zero    
% qd([1:6])=zeros(6,1);    
% qdd([1:6])=zeros(6,1);    
    
%/*-- End of user code --*/               
               
return               
