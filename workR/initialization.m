 
MBS_user.flag_grfx=zeros(13,1); 
MBS_user.flag_grfy=zeros(13,1); 
MBS_user.temp_grfx=zeros(13,1); 
MBS_user.temp_grfy=zeros(13,1); 
 
MBS_user.u=zeros(23,1); 
 
MBS_user.resdirdyn.qref=zeros(29,1); 
 
% number of joints (including 6 DoF floating base) :    
n=mbs_data.Njoint;    
 
% PD gains (all joints) 
MBS_user.Control.Kp=150; 
MBS_user.Control.Kd=15; 
 
switch type 
    case{'walk'} 
%         % LOAD WALKING TRAJ 
        load simout.anim 
        traj=simout;  
        MBS_user.traj=traj; 
 
%     case{'squat'} 
%         MBS_data.q(1:29)= zeros(29,1);  
%     case {'pushup'} 
%         % set initial conditions for a pushup motion 
%         MBS_data.q(1:29)= zeros(29,1);  
%         MBS_data.q(20)= 0.85;  
%         MBS_data.q(22)=-1.5; 
%         MBS_data.q(26)=-1.5; 
%         % ELBOW: 
%         MBS_data.q(25)=-0.5; 
%         MBS_data.q(29)=-0.5; 
%         % Hip pitch 
%         MBS_data.q(7)=0.4; 
%         MBS_data.q(13)=0.4; 
%         % body orientation/position 
%         MBS_data.q(1)= 0.35; 
%         MBS_data.q(3)= 0.35; 
%         MBS_data.q(5)= 0.75; 
%         mbs_data.q=MBS_data.q; 
%     case{'armsup'} 
%         % set initial conditions for a armsup motion 
%         MBS_data.q(7:29)= zeros(23,1);  
%          
%         MBS_data.q(22)= 0.0;  
%         MBS_data.q(26)= 0.0;  
%          
%         MBS_data.q(23)= -1.5;  
%         MBS_data.q(27)= 1.5;  
         
    otherwise 
end 
