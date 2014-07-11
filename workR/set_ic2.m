%INITIAL CONDITION :    
% defining user derivatives for the motors:   
 
 
type=MBS_user.type; 
 
% if strcmp(type, 'pushup') 
%      
%     MBS_data=mbs_set_qu(MBS_data,[1:3 5 7:29]); 
%     mbs_data=mbs_set_qu(mbs_data,[1:3 5 7:29]); 
%      
%     mbs_data=mbs_set_qdriven(mbs_data,[4 6]); 
%     MBS_data=mbs_set_qdriven(MBS_data,[4 6]); 
%     mbs_data=mbs_set_qlocked(mbs_data,[4 6]); 
%     MBS_data=mbs_set_qlocked(MBS_data,[4 6]); 
% else 
%  
%     mbs_data=mbs_set_qlocked(mbs_data,[2 4:6]); 
%     MBS_data=mbs_set_qlocked(MBS_data,[2 4:6]); 
% end 
 
mbs_data.Nux=2*23;   
 
NFBJ=6-mbs_data.nqc; 
 
y0=zeros(2*(23+NFBJ)+2*23,1);   
 
% if strcmp(type, 'pushup') 
%     y0(3)=0.4; 
%     mbs_data.q(3)=y0(3); 
%     y0(1)=MBS_data.q(1); 
%     y0(4)=1.4; 
%     mbs_data.q(5)=y0(4); 
% else 
%     y0(2)=0.56; 
% end 
 
 
y0(NFBJ+1:23+NFBJ)=[Traj(1,8:30)]'; 
 
mbs_data.q(7:29)=[Traj(1,8:30)]';  
 
mbs_data.q(3)=0.55; 
y0(3)=0.55; 
 
%position of motors  
y0(2*(23+NFBJ)+1:2*(23+NFBJ)+23)=[Traj(1,8:30)]';  
%velocity of motors  
y0(2*(23+NFBJ)+1+23:2*(23+NFBJ)+46)=zeros(23,1);  
  
MBS_user.PositionRefrences=Traj;   
