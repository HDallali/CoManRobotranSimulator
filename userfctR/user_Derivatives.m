function [uxd] = user_Derivatives(ux,mbs_data,tsim)              
% --------------------------              
% UCL-CEREM-MBS              
%              
% @version MBsysLab_m 1.7.a              
%              
% Creation : 2006              
% Last update : 30/09/2008              
% -------------------------              
%              
%[uxd] = user_Derivatives(ux,mbs_data,tsim)              
%              
% ux : user vector of state variables              
% mbs_data : multibody data structure              
% tsim : current time              
%              
% uxd : user vector of state derivatives              
              
global MBS_user MBS_info              
              
uxd = zeros(mbs_data.Nux,1);              
              
%/*-- Begin of user code --*/              
   
% u=zeros(23,1);   
 
    q=mbs_data.q(7:(23+6));   
    qd=mbs_data.qd(7:(23+6));   
    qm=mbs_data.ux(1:23);   
    qmd=mbs_data.ux(24:2*23);   
       
    x=[q;qd;qm;qmd];   
      
    JJ= MBS_user.JJdrives;   
    DD= MBS_user.DDdrives;   
    VT= MBS_user.VTgdrives;   
    KKs=MBS_user.KKs;   
    DDs=MBS_user.DDs;   
      
    u=MBS_user.u;   
    % computing motor accelerations:   
    qmdd=-inv(JJ)*[-KKs -DDs KKs DDs+DD]*x+inv(JJ)*u;   
    % returning the derivatives:   
    uxd=[qmd;qmdd];   
%/*-- End of user code --*/              
              
return    
