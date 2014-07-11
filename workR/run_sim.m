% Sample time:    
Ts=1e-3;    
MBS_user.Ts=Ts;    
% Starting time:    
t0=0;    
% Max simulation time:    
max_time=t_max;    
 
% Integration setting:    
dtmax=0.001;  
% reltol=1.0000e-002;  
% abstol=1.0000e-002;   
options = odeset('MaxStep',dtmax);   
% options = odeset('MaxStep',dtmax);   
% options = odeset(options,'Events', @(t,y,callfct) EventDetectionFall(t, y, callfct, mbs_info, mbs_data));   
options = odeset(options,'Outputfcn','mbs_out_dirdyn');   
  
[MBS_dirdyn] = mbs_new_dirdyn(mbs_data);    
yprime = str2func(MBS_dirdyn.fctDerivatives);    
callfct = mbs_get_fct_handle(MBS_dirdyn.fctname);    
    
tspan= t0:Ts:max_time;     
nt=length(tspan);    
 
% integrate for Ts seconds     
% [t, y] = ode45(yprime,tspan,y0,options,callfct,0); %RK45     
% (ode45 is slow in stiff cases)    
 
% integrate for Ts seconds    
[t, y]  = ode15s(yprime,tspan,y0,options,callfct,0); % RK15s (stiff case)   
% [t, y, TE, YE, IE]  = ode45(yprime,tspan,y0,options,callfct,0); % RK15s (stiff case)   
 
% [t, y] = ode23s(yprime,tspan,y0,options,callfct,0); % RK15s (stiff case)    
   
 
tt=0:0.001:(size(y,1)-1)*0.001;   
 
ns=length(tt);  
 
Traj_out=[tt' zeros(ns,29)];   
Traj_out(:,4)=1.0; 
 
Traj_out(:,[1+mbs_data.qu])=y(:,1:mbs_data.nqu) ; 
 
cd .. 
cd animationR   
writemat2('simout.anim',Traj_out)   
cd ..   
cd workR    
  
