% Plot the results 
 
 
% get the sent references: 
qref=[MBS_user.resdirdyn.qref]'; 
 
 figure(1)    
% Angle: right leg    
subplot(3,2,1), plot(MBS_dirdyn.tsim,MBS_dirdyn.q(:,1+6), MBS_dirdyn.tsim,qref(:,1+6)),grid on,xlabel('t (sec)'),ylabel('Angle RHipS'); title('Joint Pos')    
subplot(3,2,3), plot(MBS_dirdyn.tsim,MBS_dirdyn.q(:,2+6), MBS_dirdyn.tsim,qref(:,2+6)),grid on,xlabel('t (sec)'),ylabel('Angle RHipL');     
subplot(3,2,5), plot(MBS_dirdyn.tsim,MBS_dirdyn.q(:,3+6), MBS_dirdyn.tsim,qref(:,3+6)),grid on,xlabel('t (sec)'),ylabel('Angle RHipY');     
subplot(3,2,2), plot(MBS_dirdyn.tsim,MBS_dirdyn.q(:,4+6), MBS_dirdyn.tsim,qref(:,4+6)),grid on,xlabel('t (sec)'),ylabel('Angle Rknee');     
subplot(3,2,4), plot(MBS_dirdyn.tsim,MBS_dirdyn.q(:,5+6), MBS_dirdyn.tsim,qref(:,5+6)),grid on,xlabel('t (sec)'),ylabel('Angle RankleL');     
subplot(3,2,6), plot(MBS_dirdyn.tsim,MBS_dirdyn.q(:,6+6), MBS_dirdyn.tsim,qref(:,6+6)),grid on,xlabel('t (sec)'),ylabel('Angle RankleS');     
xlabel('t (sec)'); ylabel('rad')   
    
figure(2)    
% Angles: left leg    
subplot(3,2,1), plot(MBS_dirdyn.tsim,MBS_dirdyn.q(:,1+12), MBS_dirdyn.tsim,qref(:,1+12)),grid on,xlabel('t (sec)'),ylabel('Angle LHipS'); title('Joint Pos')       
subplot(3,2,3), plot(MBS_dirdyn.tsim,MBS_dirdyn.q(:,2+12), MBS_dirdyn.tsim,qref(:,2+12)),grid on,xlabel('t (sec)'),ylabel('Angle LHipL');     
subplot(3,2,5), plot(MBS_dirdyn.tsim,MBS_dirdyn.q(:,3+12), MBS_dirdyn.tsim,qref(:,3+12)),grid on,xlabel('t (sec)'),ylabel('Angle LHipY');     
subplot(3,2,2), plot(MBS_dirdyn.tsim,MBS_dirdyn.q(:,4+12), MBS_dirdyn.tsim,qref(:,4+12)),grid on,xlabel('t (sec)'),ylabel('Angle Lknee');     
subplot(3,2,4), plot(MBS_dirdyn.tsim,MBS_dirdyn.q(:,5+12), MBS_dirdyn.tsim,qref(:,5+12)),grid on,xlabel('t (sec)'),ylabel('Angle LankleL');     
subplot(3,2,6), plot(MBS_dirdyn.tsim,MBS_dirdyn.q(:,6+12), MBS_dirdyn.tsim,qref(:,6+12)),grid on,xlabel('t (sec)'),ylabel('Angle LankleS');    
xlabel('t (sec)'); ylabel('rad') 
 
figure(3) 
subplot(3,1,1), plot(MBS_dirdyn.tsim,MBS_dirdyn.q(:,7+12), MBS_dirdyn.tsim, qref(:,7+12)),grid on,xlabel('t (sec)'),ylabel('Waist roll'); title('Joint Pos')       
subplot(3,1,2), plot(MBS_dirdyn.tsim,MBS_dirdyn.q(:,8+12), MBS_dirdyn.tsim, qref(:,8+12)),grid on,xlabel('t (sec)'),ylabel('Waist pitch');     
subplot(3,1,3), plot(MBS_dirdyn.tsim,MBS_dirdyn.q(:,9+12), MBS_dirdyn.tsim, qref(:,9+12)),grid on,xlabel('t (sec)'),ylabel('Waist Yaw');     
xlabel('t (sec)'); ylabel('rad') 
% set initial conditions for a pushup motion 
if strcmp(type,'pushup')   
     
    figure(10) 
    subplot(2,2,1), plot(MBS_dirdyn.tsim,MBS_dirdyn.q(:,22), MBS_dirdyn.tsim, qref(:,22)),grid on,xlabel('t (sec)'),ylabel('Right Shoulder Pitch'); 
    title('Joint Pos')    ; xlabel('t (sec)'); ylabel('rad') 
    subplot(2,2,2), plot(MBS_dirdyn.tsim,MBS_dirdyn.q(:,25), MBS_dirdyn.tsim, qref(:,25)),grid on,xlabel('t (sec)'),ylabel('Right Elbow');     
    subplot(2,2,3), plot(MBS_dirdyn.tsim,MBS_dirdyn.q(:,26), MBS_dirdyn.tsim, qref(:,26)),grid on,xlabel('t (sec)'),ylabel('Left Shoulder Pitch');     
    subplot(2,2,4), plot(MBS_dirdyn.tsim,MBS_dirdyn.q(:,29), MBS_dirdyn.tsim, qref(:,29)),grid on,xlabel('t (sec)'),ylabel('Left Elbow');     
     
end 
  
%% Plotting joints torques   
figure(4)    
% Torques: floating base   
subplot(3,2,1), plot(MBS_dirdyn.tsim,MBS_dirdyn.Qq(:,1)),grid on,xlabel('t (sec)'),ylabel('Floating base DoF Torque T_1'); title('Joint Torque')   
xlabel('t (sec)'); ylabel('Nm') 
subplot(3,2,3), plot(MBS_dirdyn.tsim,MBS_dirdyn.Qq(:,2)),grid on,xlabel('t (sec)'),ylabel('Floating base DoF Torque T_2');     
subplot(3,2,5), plot(MBS_dirdyn.tsim,MBS_dirdyn.Qq(:,3)),grid on,xlabel('t (sec)'),ylabel('Floating base DoF Torque T_3');     
  
subplot(3,2,2), plot(MBS_dirdyn.tsim,MBS_dirdyn.Qq(:,4)),grid on,xlabel('t (sec)'),ylabel('Floating base DoF Torque R_1');     
subplot(3,2,4), plot(MBS_dirdyn.tsim,MBS_dirdyn.Qq(:,5)),grid on,xlabel('t (sec)'),ylabel('Floating base DoF Torque R_2');     
subplot(3,2,6), plot(MBS_dirdyn.tsim,MBS_dirdyn.Qq(:,6)),grid on,xlabel('t (sec)'),ylabel('Floating base DoF Torque R_3');     
    
    
figure(5)    
% Torques: right leg    
subplot(3,2,1), plot(MBS_dirdyn.tsim,MBS_dirdyn.Qq(:,1+6)),grid on,xlabel('t (sec)'),ylabel('Torque RHipS'); title('Joint Torque')   
xlabel('t (sec)'); ylabel('Nm') 
subplot(3,2,3), plot(MBS_dirdyn.tsim,MBS_dirdyn.Qq(:,2+6)),grid on,xlabel('t (sec)'),ylabel('Torque RHipL');     
subplot(3,2,5), plot(MBS_dirdyn.tsim,MBS_dirdyn.Qq(:,3+6)),grid on,xlabel('t (sec)'),ylabel('Torque RHipY');     
subplot(3,2,2), plot(MBS_dirdyn.tsim,MBS_dirdyn.Qq(:,4+6)),grid on,xlabel('t (sec)'),ylabel('Torque Rknee');     
subplot(3,2,4), plot(MBS_dirdyn.tsim,MBS_dirdyn.Qq(:,5+6)),grid on,xlabel('t (sec)'),ylabel('Torque RankleL');     
subplot(3,2,6), plot(MBS_dirdyn.tsim,MBS_dirdyn.Qq(:,6+6)),grid on,xlabel('t (sec)'),ylabel('Torque RankleS');     
    
    
figure(6)    
% Torques: left leg    
subplot(3,2,1), plot(MBS_dirdyn.tsim,MBS_dirdyn.Qq(:,1+12)),grid on,xlabel('t (sec)'),ylabel('Torque LHipS');  title('Joint Torque')     
xlabel('t (sec)'); ylabel('Nm') 
subplot(3,2,3), plot(MBS_dirdyn.tsim,MBS_dirdyn.Qq(:,2+12)),grid on,xlabel('t (sec)'),ylabel('Torque LHipL');     
subplot(3,2,5), plot(MBS_dirdyn.tsim,MBS_dirdyn.Qq(:,3+12)),grid on,xlabel('t (sec)'),ylabel('Torque LHipY');     
subplot(3,2,2), plot(MBS_dirdyn.tsim,MBS_dirdyn.Qq(:,4+12)),grid on,xlabel('t (sec)'),ylabel('Torque Lknee');     
subplot(3,2,4), plot(MBS_dirdyn.tsim,MBS_dirdyn.Qq(:,5+12)),grid on,xlabel('t (sec)'),ylabel('Torque LankleL');     
subplot(3,2,6), plot(MBS_dirdyn.tsim,MBS_dirdyn.Qq(:,6+12)),grid on,xlabel('t (sec)'),ylabel('Torque LankleS');     
  
%% ground reaction force:  
figure(7),  
Fz_r=squeeze(sum(MBS_user.resdirdyn.GRF(3,1:5,:))); 
Fz_l=squeeze(sum(MBS_user.resdirdyn.GRF(3,6:10,:))); 
 
subplot(2,1,1),plot(MBS_user.resdirdyn.tsim,Fz_r); legend('Right');grid;  title('Joint Torque')     
title('Normal ground reaction forces'); xlabel('t (sec)'); ylabel('N') 
subplot(2,1,2),plot(MBS_user.resdirdyn.tsim,Fz_l);  
legend('Left'); xlabel('t (sec)'); ylabel('N') 
 
grid;  
axis('tight')   
  
