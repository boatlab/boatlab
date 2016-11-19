function [sys,x0,str,ts] = DiscKal(t,x,u,flag,data)
% Shell for the discrete kalman filter assignment in
% TTK4115 Linear Systems.
%
% Author: Jørgen Spjøtvold
% 19/10-2003 

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes(data);

  %%%%%%%%%%%%%
  % Outputs   %
  %%%%%%%%%%%%%
  
  case 3,
    sys=mdlOutputs(t,x,u,data);
  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  
  case 2,
    sys=mdlUpdate(t,x,u,data);
  
  case {1,4,}
    sys=[];

  case 9,
      sys=mdlTerminate(t,x,u);
  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    error(['Unhandled flag = ',num2str(flag)]);
end

function [sys,x0,str,ts]=mdlInitializeSizes(data)
% This is called only at the start of the simulation. 

sizes = simsizes; % do not modify

sizes.NumContStates  = 0; % Number of continuous states in the system, do not modify
sizes.NumDiscStates  = 35; % Number of discrete states in the system, modify. 
sizes.NumOutputs     = 3; % Number of outputs, the hint states 2
sizes.NumInputs      = 2; % Number of inputs, the hint states 2
sizes.DirFeedthrough = 1; % 1 if the input is needed directly in the
% update part
sizes.NumSampleTimes = 1; % Do not modify  

sys = simsizes(sizes); % Do not modify  

str = []; % Do not modify

%Time step 10 Hz
T_sample = 0.1;

ts  = [T_sample 0]; % Sample time. [-1 0] means that sampling is
% inherited from the driving block and that it changes during
% minor steps.

x0 = [data.xm0 data.xh0 data.Pm0(:)']; % Initial values for the discrete states, modify

function sys=mdlUpdate(t,x,u,data)%mdlUpdate(t,x,u, data); if method 2 is used
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Update the filter covariance matrix and state etsimates here.
% example: sys=x+u(1), means that the state vector after
% the update equals the previous state vector + input nr one.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Pm_vec = x(11:35);
Pm = reshape(Pm_vec,sqrt(length(Pm_vec)),sqrt(length(Pm_vec)));

%% Kalman filter update
%(1) Kalman gain
L = Pm*(data.C)'*inv(data.C*Pm*(data.C)'+data.R);
%(2) Update estimate
x(6:10) = x(1:5) + L*(u(2)-(data.C*x(1:5)));
%(3) Update covariance
%P = (eye(5)-L*C)*Pm*(eye(5)-L*C)' + L*R*L';
P = (eye(5)- (L*data.C))*Pm;
x(11:35) = P(:)';

%(4) Project ahead
x(1:5) = data.A*x(6:10) + data.B*u(1);
P = reshape(x(11:35), sqrt(length(x(11:35))), sqrt(length(x(11:35))));
Pm = data.A*P*(data.A)' + data.Q;
x(11:35) = Pm(:)';
sys=x;
      
function sys=mdlOutputs(t,x,u,data)% mdlOutputs(t,x,u,data) if mathod 2 is used
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate the outputs here
% example: sys=x(1)+u(2), means that the output is the first state+
% the second input. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sys=[x(10) x(8) x(7)];

function sys=mdlTerminate(t,x,u) 
sys = [];
