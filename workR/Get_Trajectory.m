function Traj = Get_Trajectory(amp, freq, t_max, type, MBS_data)   
 
global MBS_user 
 
t=[0:0.001:t_max]'; 
f=freq; 
 
 
 
ns=length(t); 
 
Traj=[t zeros(ns,29)]; 
 
switch (type) 
    case{'walk'} 
        % LOAD WALKING TRAJ 
        load simout.anim 
        traj=simout;  
        MBS_user.traj=traj; 
        Traj=traj; 
         
    case{'squat'} 
        q1=  -amp*(1-cos(f*2*pi*t)); 
        q2= 2*amp*(1-cos(f*2*pi*t)); 
        q3=  -amp*(1-cos(f*2*pi*t)); 
 
        % squat motion 
        Traj(:, 1+ [7 10 12 13 16 18]) = [q1 q2 q3 q1 q2 q3]; 
         
    case{'pushup'} 
        q22= -0.4 -amp*(1-cos(f*2*pi*t));  
        q26= -0.4 -amp*(1-cos(f*2*pi*t));  
 
        q25= -1.5-2*amp*(1-cos(f*2*pi*t)); 
        q29= -1.5-2*amp*(1-cos(f*2*pi*t)); 
                     
        Traj(:, 1+ [22 25 26 29 ]) = [q22 q25 q26 q29]; 
         
    case{'armsup'} 
        q23= MBS_data.q(23)+ 2*amp*(1-cos(f*2*pi*t));  
        q27= MBS_data.q(27)-2*amp*(1-cos(f*2*pi*t));  
         
        q22= -0.4 -amp*(1-cos(f*2*pi*t));  
        q26= -0.4 -amp*(1-cos(f*2*pi*t));  
         
        q24=  amp*(sin(f*2*pi*t));  
        q28=  amp*(sin(f*2*pi*t));  
         
        q25= -1.0-amp*(1-cos(f*2*pi*t)); 
        q29= -1.0-amp*(1-cos(f*2*pi*t)); 
             
        Traj(:, 1+ [22 23 24 25 26 27 28 29]) = [q22 q23 q24 q25 q26 q27 q28 q29]; 
    otherwise 
       error('Please specify the type of trajectory you want, i.e. squat, sway, ... ') 
end 
 
