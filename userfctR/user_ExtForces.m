function Swr = user_ExtForces(PxF,RxF,VxF,OMxF,AxF,OMPxF,mbs_data,tsim,ixF)               
% --------------------------               
% UCL-CEREM-MBS               
%               
% @version MBsysLab_m 1.7.a               
%               
% Creation : 2006               
% Last update : 30/09/2008               
% -------------------------               
%               
%Swr = user_ExtForces(PxF,RxF,VxF,OMxF,AxF,OMPxF,mbs_data,tsim,ixF)               
%               
% PxF(3,1) : absolute position vector of the external force application point                
% RxF(3,3) : absolute rotation matrix of the body               
% VxF(3,1) : absolute velocity vector of the external force application point                
% OMxF(3,1) : absolute angular velocity vector of the body               
% AxF(3,1) : absolute acceleration vector of the external force application point                
% OMPxF(3,1) : absolute angular acceleration vector of the body               
%               
% => All above vectors are expressed in the inertial reference frame !               
%               
% mbs_data : multibody data structure               
% tsim : current time               
% ixF : index of the external force sensor ('F' type in MBsysPad)               
%        (can be obtained via the 'mbs_get_F_sensor_id' function)               
%               
% Swr(9,1) = [Fx; Fy; Fz; Mx; My; Mz; dxF];               
%   - Force components (expressed in the inertial frame) : Fx, Fy, Fz               
%   - Pure torque components (expressed in the inertial frame) : Mx, My, Mz               
%   - Application point local coordinates vector (expressed in the body-fixed frame) : dxF(1:3,1);               
%               
% this function may use a global structure called MBS_user               
               
global MBS_user MBS_info               
    
    
Fx=0.0; Fy=0.0; Fz=0.0;               
Mx=0.0; My=0.0; Mz=0.0;               
idpt = mbs_data.xfidpt(ixF);               
dxF = mbs_data.dpt(:,idpt);            
 
         
%/*-- Begin of user code --*/               
%                
% Use the 'mbs_get_F_sensor_id' function to get easily the force sensor               
% indices, e.g. :               
% F1 = mbs_get_F_sensor_id(MBS_info,'LHand');               
%           eg.       
%  F_midwaist = mbs_get_F_sensor_id(MBS_info,'ExtForce_MidWaist');                
%  F_TorsoCg = mbs_get_F_sensor_id(MBS_info,'ExtForce_TorsoCg');                 
   
  
mu=MBS_user.mu_grf;     
 
  
threshold=-1e-9 ;  
 
if (PxF(3)<=threshold && ixF<=13) % dis-activated the upper body to  
%stick the feet to the ground.   
        % apply the linear model for the normal force:  
            Fz= -MBS_user.ContactCof(1)*PxF(3) - MBS_user.ContactCof(2)*VxF(3);   
 
        if Fz>0     
 
        else     
           Fz=0;     
        end     
        %----------------------------------------------------------                     
        % Horizontal friction force in X direction:     
        if MBS_user.flag_grfx(ixF)==0     
           MBS_user.temp_grfx(ixF)=PxF(1); %store the initial point of contact.     
           MBS_user.flag_grfx(ixF)=1;     
        end     
 
        Fx=-MBS_user.ContactCof(2)*VxF(1)-MBS_user.ContactCof(1)*(PxF(1)-MBS_user.temp_grfx(ixF)); % friction force can be positive or negative.     
 
        %----------------------------------------------------------     
        % Horizontal friction force in Y direction:     
        if MBS_user.flag_grfy(ixF)==0     
           MBS_user.temp_grfy(ixF)=PxF(2); %store the initial point of contact.     
           MBS_user.flag_grfy(ixF)=1;     
        end     
 
        Fy=-MBS_user.ContactCof(2)*VxF(2)-MBS_user.ContactCof(1)*(PxF(2)-MBS_user.temp_grfy(ixF)); % friction force can be positive or negative.     
 
   if (ixF<=10) % apply sliding only for the feet, not the upper body. 
         fr=sqrt(Fx^2+Fy^2);    
         if fr>mu*Fz    
             fr=mu*Fz;         
             Alpha=atan2(VxF(2),VxF(1));    
             Fx=-fr*cos(Alpha);    
             Fy=-fr*sin(Alpha);    
             MBS_user.flag_grfx(ixF)=0;     
             MBS_user.flag_grfy(ixF)=0;     
         end    
   end 
 
else     
    Fx=0;     
    Fy=0;     
    Fz=0;     
    MBS_user.flag_grfx(ixF)=0;     
    MBS_user.flag_grfy(ixF)=0;     
end;     
 
%/*-- End of user code --*/               
% end               
   
  
  
%extForces storage for animation               
% position of the force in the inertial frame     
MBS_user.curvar.extForces(ixF).P = (PxF'+RxF'*dxF)';               
% rotation matrix between inertal and user frame (default: no rotation)               
MBS_user.curvar.extForces(ixF).R = [1 0 0; 0 1 0; 0 0 1];               
% Force components in the user frame (default: in the inertial frame)               
MBS_user.curvar.extForces(ixF).F = [Fx Fy Fz];               
               
             
Swr = [Fx; Fy; Fz; Mx; My; Mz; dxF];        
   
MBS_user.GRF(:,ixF)=Swr(1:3);   
   
MBS_user.GRFPxF(:,ixF)=[Fx*PxF(1); Fy*PxF(2)];    
   
return               
