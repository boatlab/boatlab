function [sys,x0,str,ts] = DiscKal(t,x,u,flag)
% Shell for the discrete kalman filter assignment in
% TTK4115 Linear Systems.
%
% Author: J�rgen Spj�tvold
% 19/10-2003 

global data A B C E R Q P Pm xm;
%{
    f1 = 'A'; v1 = A;
    f2 = 'B'; v2 = B;
    f3 = 'C'; v3 = C;
    f4 = 'E'; v4 = E;
    f5 = 'R'; v5 = R;
    f6 = 'Q'; v6 = Q;
    f7 = 'P'; v7 = P;
    f8 = 'Pm'; v8 = Pm;
    f9 = 'xm'; v9 = xm;
    data = struct(f1,v1,f2,v2,f3,v3,f4,v4,f5,v5,f6,v6,f7,v7,f8,v8,f9,v9);

%}
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
sizes.NumDiscStates  = 5; % Number of discrete states in the system, modify. 
sizes.NumOutputs     = 3; % Number of outputs, the hint states 2
sizes.NumInputs      = 2; % Number of inputs, the hint states 2
sizes.DirFeedthrough = 1; % 1 if the input is needed directly in the
% update part
sizes.NumSampleTimes = 1; % Do not modify  

sys = simsizes(sizes); % Do not modify  

str = []; % Do not modify

ts  = [-1 0]; % Sample time. [-1 0] means that sampling is
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

%Time step 10 Hz
T = 0.1;

% Using Van Loan's method to derive A, B and Q
AQ_exponent = [A_cont, E_cont*Q_cont*transp(E_cont); zeros(5,5), -transp(A_cont)];
AQ = exp((AQ_exponent * T));

AB_exponent = [A_cont, B_cont; zeros(5,5), zeros(5,1)];
AB = exp((AB_exponent * T));

data.A = AQ(1:5,1:5)
data.B = AB(1:5,6)
data.Q = AQ(1:5,6:10)*transp(AQ(1:5,1:5))

% C is the same
data.C = C_cont

% R_cont is found using var() on the output from model with zero input.
R_cont = 0.002;
% R is averaged R_cont
data.R = R_cont/T


data.P = [
    1,      0,      0,      0,      0;
    0,      0.013,  0,      0,      0;
    0,      0,      pi^2,   0,      0;
    0,      0,      0,      1,      0;
    0,      0,      0,      0,      2.5*10^(-4)
    ]

data.Pm = [
    1,      0,      0,      0,      0;
    0,      0.013,  0,      0,      0;
    0,      0,      pi^2,   0,      0;
    0,      0,      0,      1,      0;
    0,      0,      0,      0,      2.5*10^(-4)
    ]

x0 = [0; 0; 0; 0; 0] % Initial values for the discrete states, modify
data.xm = [0; 0; 0; 0; 0]

function sys=mdlUpdate(t,x,u,data)%mdlUpdate(t,x,u, data); if method 2 is used
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Update the filter covariance matrix and state etsimates here.
% example: sys=x+u(1), means that the state vector after
% the update equals the previous state vector + input nr one.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Kalman filter update
%(1) Kalman gain
L = data.Pm*data.C'*inv(data.C*data.Pm*data.C'+data.R)
%(2) Update estimate
xh = data.xm + L*(u(2)-data.C*data.xm)
%(3) Update covariance
width_p = numel(data.P)
height_p = numel(data.xm)
data.P = (eye(5)-L*data.C)*data.Pm*(eye(5)-L*data.C)' + L*data.R*L'
%(4) Project ahead
data.xm = data.A*xh + data.B*u(1)
data.Pm = data.A*data.P*data.A' +data.Q
sys=data.xm



      
function sys=mdlOutputs(t,x,u,data)% mdlOutputs(t,x,u,data) if mathod 2 is used
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Calculate the outputs here
% example: sys=x(1)+u(2), means that the output is the first state+
% the second input. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
sys=[x(5),x(3),x(2)];

function sys=mdlTerminate(t,x,u) 
sys = [];


