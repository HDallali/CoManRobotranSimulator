function [Jm,Km,cm,VTgain]=RBE1211_CSD17_CoMan_drive(option)    
%Kollmorgen motors RBE 1211 A windings    
%gearbox Harmonic Drives CSD 17-2A 100:1    
%motor position and velocity reflected to load side (CoMan links)    
    
%option = 0 for motor with Harmonic Drive only   
%option = 1 for motor with Harmonic Drive and passive compliance   
%option = 2 for motor with Harmonic Drive, additional reduction stage and passive compliance (compliance after additional reduction stage)   
%option = 3 for motor with Harmonic Drive, additional reduction stage    
%option = 4 for motor with Harmonic Drive, additional reduction stage and passive compliance (compliance after Harmonic Drive)   
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
% %Below is the code to call the M-file RBE1211_CSD17_CoMan_drive with the correct options for CoMan   
% %   
% %ankle sagittal motion right (pitch, y-rotation in Robotran)    
% [Jm_R_anksag,Ksm_R_anksag,Dm_R_anksag,Vm_R_anksag]=RBE1211_CSD17_CoMan_drive(1);    %Harmonic drive and compliance   
% %ankle lateral motion right (roll, x-rotation in Robotran)    
% [Jm_R_anklat,Ksm_R_anklat,Dm_R_anklat,Vm_R_anklat]=RBE1211_CSD17_CoMan_drive(3);    %Harmonic drive and stiff reducer   
% %knee sagittal motion right (pitch, y-rotation in Robotran)    
% [Jm_R_knesag,Ksm_R_knesag,Dm_R_knesag,Vm_R_knesag]=RBE1211_CSD17_CoMan_drive(1);    %Harmonic drive and compliance   
% %hip yaw motion right (yaw, z-rotation in Robotran)    
% [Jm_R_hipyaw,Ksm_R_hipyaw,Dm_R_hipyaw,Vm_R_hipyaw]=RBE1211_CSD17_CoMan_drive(0);    %Harmonic drive only   
% %hip lateral motion right (roll, x-rotation in Robotran)    
% [Jm_R_hiplat,Ksm_R_hiplat,Dm_R_hiplat,Vm_R_hiplat]=RBE1211_CSD17_CoMan_drive(3);    %Harmonic drive and stiff reducer   
% %hip sagittal motion right (pitch, y-rotation in Robotran)    
% [Jm_R_hipsag,Ksm_R_hipsag,Dm_R_hipsag,Vm_R_hipsag]=RBE1211_CSD17_CoMan_drive(1);    %Harmonic drive and compliance   
%    
% %hip sagittal motion left leg (pitch, y-rotation in Robotran)    
% [Jm_L_hipsag,Ksm_L_hipsag,Dm_L_hipsag,Vm_L_hipsag]=RBE1211_CSD17_CoMan_drive(1);     %Harmonic drive and compliance   
% %hip lateral motion left leg (roll, x-rotation in Robotran)    
% [Jm_L_hiplat,Ksm_L_hiplat,Dm_L_hiplat,Vm_L_hiplat]=RBE1211_CSD17_CoMan_drive(3);     %Harmonic drive and stiff reducer   
% %hip yaw motion left (yaw, z-rotation in Robotran)    
% [Jm_L_hipyaw,Ksm_L_hipyaw,Dm_L_hipyaw,Vm_L_hipyaw]=RBE1211_CSD17_CoMan_drive(0);    %Harmonic drive only   
% %knee sagittal motion left leg (pitch, y-rotation in Robotran)    
% [Jm_L_knesag,Ksm_L_knesag,Dm_L_knesag,Vm_L_knesag]=RBE1211_CSD17_CoMan_drive(1);     %Harmonic drive and compliance   
% %ankle lateral motion left leg (roll, x-rotation in Robotran)    
% [Jm_L_anklat,Ksm_L_anklat,Dm_L_anklat,Vm_L_anklat]=RBE1211_CSD17_CoMan_drive(3);     %Harmonic drive and stiff reducer   
% %ankle sagittal motion left leg (pitch, y-rotation in Robotran)    
% [Jm_L_anksag,Ksm_L_anksag,Dm_L_anksag,Vm_L_anksag]=RBE1211_CSD17_CoMan_drive(1);     %Harmonic drive and compliance   
%    
% %waist lateral motion (roll, x-rotation in Robotran)    
% [Jm_waistlat,Ksm_waistlat,Dm_waistlat,Vm_waistlat]=RBE1211_CSD17_CoMan_drive(0);     %Harmonic drive only   
% %waist sagittal motion (pitch, y-rotation in Robotran)    
% [Jm_waistsag,Ksm_waistsag,Dm_waistsag,Vm_waistsag]=RBE1211_CSD17_CoMan_drive(1);     %Harmonic drive and compliance   
% %waist yaw motion (yaw, z-rotation in Robotran)    
% [Jm_waistyaw,Ksm_waistyaw,Dm_waistyaw,Vm_waistyaw]=RBE1211_CSD17_CoMan_drive(1);     %Harmonic drive and compliance   
   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
   
N=100;                          %Harmonic Drive reduction ratio    
    
Jm=(5.4e-6+8.47e-6)*(N^2);      %kg*m^2 gearbox+motor inertia reflected to load    
    
Km=0.84e4;                      %N-m/rad gearbox stiffness minimum    
%Km=1.30e4;                     %N-m/rad gearbox stiffness maximum    
        
Lm=0.32e-3;                     %Henries motor inductance   
Rm=0.664;                       %ohms    motor resistance    
Ktor=4.10e-2;                   %Nm/A    motor torque constant    
Kbemf=4.10e-2;                  %Vs/rad  motor bemf constant    
cm=1.9958e-5*(N^2);             %Nms/rad motor viscous friction reflected to load    
cm=cm+Ktor*Kbemf/Rm*(N^2);      %adding damping due to bemf     
VTgain=N*Ktor/Rm;               %voltage to torque gain (neglecting inductance)   
    
    
%%%%%%%%% compliant joint for CoMan at the output of Harmonic Drive    
if(option==1)    
    Kcompjoint=415;                              %N-m/rad  theoretical value        
    %equivalent stiffness     
    Kme=inv(1/Km+1/Kcompjoint);    
    Km=Kme;    
end    
    
%%%%%%%% compliant joint for CoMan and aditional reduction stage    
%%%%%%%% compliance at the output of the additional reduction stage    
if(option==2)    
    Nr=1.0;                                      %   
    Kcompjoint=415;                              %N-m/rad  theoretical value     
    Km=Km*(Nr^2);                                %reflected motor stiffness at reducer output    
    %equivalent stiffness     
    Kme=inv(1/Km+1/Kcompjoint);    
    %reflected parameters at reducer output    
    Jm=Jm*(Nr^2);    
    Km=Kme;    
    cm=cm*(Nr^2);    
    VTgain=VTgain*Nr;    
end    
    
%%%%%%%% joint for CoMan with Harmonic Drive and additional reduction stage   
%%%%%%%% Need stiffness of additional reducer (should be estimated from experimental tests)   
if(option==3)    
    Nr=1.0;                                      %   
    Kreducer=1e12;                               %N-m/rad  theoretical value        
    %equivalent stiffness     
    Kme=inv(1/Km+1/Kreducer);    
    %reflected parameters at reducer output    
    Jm=Jm*(Nr^2);    
    Km=Kme*(Nr^2);    
    cm=cm*(Nr^2);    
    VTgain=VTgain*Nr;    
end    
   
%%%%%%%% compliant joint for CoMan at the output of Harmonic Drive     
%%%%%%%% and additional reduction stage after compliance   
if(option==4)    
    Nr=1.0;                                      %   
    Kcompjoint=415;                              %N-m/rad  theoretical value        
    %equivalent stiffness     
    Kme=inv(1/Km+1/Kcompjoint);    
    %reflected parameters at reducer output    
    Jm=Jm*(Nr^2);    
    Km=Kme*(Nr^2);    
    cm=cm*(Nr^2);    
    VTgain=VTgain*Nr;    
end    
   
