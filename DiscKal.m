function [sys,x0,str,ts] = DiscKal(t,x,u,flag)
% Shell for the discrete kalman filter assignment in
% TTK4115 Linear Systems.
%
% Author: Jørgen Spjøtvold
% 19/10-2003 

global A B C R Q P Pm xm;
switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes;

  %%%%%%%%%%%%%
  % Outputs   %
  %%%%%%%%%%%%%
  
  case 3,
    sys=mdlOutputs(t,x,u);
  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  
  case 2,
    sys=mdlUpdate(t,x,u);
  
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

function [sys,x0,str,ts]=mdlInitializeSizes
% This is called only at the start of the simulation. 
global A B C R Q P Pm xm;
sizes = simsizes; % do not modify

sizes.NumContStates  = 0; % Number of continuous states in the system, do not modify
sizes.NumDiscStates  = 5; % Number of discrete states in the system, modify. 
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

w_0 = 0.4895;
lambda = 0.0657;
K = 0.1744;
T = 87.1542;
K_w = 0.1999;

% Contiuous model
A_cont = [ 
0,           1,                       0,   0,       0;
-w_0*w_0,    -2*lambda*w_0*w_0,   0,   0,       0;
0,           0,                       0,   1,       0;
0,           0,                       0,   -1/T,    -K/T;
0,           0,                       0,   0,       0
];

B_cont = [
    0; 
    0; 
    0; 
    K/T; 
    0
];

C_cont = [0, 1, 1, 0, 0];

E_cont = [
    0, 0;
    K_w, 0;
    0, 0;
    0, 0;
    0, 1
]

%Process noise covariance matrix
Q_cont = [30, 0; 0, 10^-6];
% Discrete system

% Using Van Loan's method to derive A, B and Q
AQ_exponent = [A_cont, E_cont*Q_cont*transp(E_cont); zeros(5,5), -transp(A_cont)];
AQ = exp((AQ_exponent * T_sample));

AB_exponent = [A_cont, B_cont; zeros(5,5), zeros(5,1)];
AB = exp((AB_exponent * T_sample));

A = AQ(1:5,1:5);
B = AB(1:5,6);
Q = AQ(1:5,6:10)*transp(AQ(1:5,1:5));

% C is the same
C = C_cont;

% R_cont is found using var() on the output from model with zero input.
R_cont = 0.002;
% R is averaged R_cont
R = R_cont/T_sample;


P = [
    1,      0,      0,      0,      0;
    0,      0.013,  0,      0,      0;
    0,      0,      pi^2,   0,      0;
    0,      0,      0,      1,      0;
    0,      0,      0,      0,      2.5*10^(-4)
    ];

Pm = [
    1,      0,      0,      0,      0;
    0,      0.013,  0,      0,      0;
    0,      0,      pi^2,   0,      0;
    0,      0,      0,      1,      0;
    0,      0,      0,      0,      2.5*10^(-4)
    ];

x0 = [0; 0; 0; 0; 0]; % Initial values for the discrete states, modify
xm = [0; 0; 0; 0; 0];

function sys=mdlUpdate(t,x,u)%mdlUpdate(t,x,u, data); if method 2 is used
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Update the filter covariance matrix and state etsimates here.
% example: sys=x+u(1), means that the state vector after
% the update equals the previous state vector + input nr one.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
global A B C R Q P Pm xm;

%% Kalman filter update
%(1) Kalman gain
L = Pm*C'*inv(C*Pm*C'+R);
%(2) Update estimate
xh = xm + L*(u(2)-C*xm);
%(3) Update covariance
P = (eye(5)-L*C)*Pm*(eye(5)-L*C)' + L*R*L';
%(4) Project ahead
xm = A*xh + B*u(1);
Pm = A*P*A' +Q;
sys=xm;



      
function sys=mdlOutputs(t,x,u)% mdlOutputs(t,x,u,data) if mathod 2 is used
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate the outputs here
% example: sys=x(1)+u(2), means that the output is the first state+
% the second input. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sys=[x(5) x(3) x(2)];

function sys=mdlTerminate(t,x,u) 
sys = [];


