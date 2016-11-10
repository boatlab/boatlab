
%persistent  A B C E R Q P Pm xm;
A = 0;
B= 0;
C= 0;
E=0;
R=0;
Q=0;
P=0;
Pm=0;
xm=0;
u=[1,1];
x=[1;1;1;1;1];
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
];

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

data.A = AQ(1:5,1:5);
data.B = AB(1:5,6);
data.Q = AQ(1:5,6:10)*transp(AQ(1:5,1:5));

% C is the same
data.C = C_cont;

% R_cont is found using var() on the output from model with zero input.
R_cont = 0.002;
% R is averaged R_cont
data.R = R_cont/T;


data.P = [
    1,      0,      0,      0,      0;
    0,      0.013,  0,      0,      0;
    0,      0,      pi^2,   0,      0;
    0,      0,      0,      1,      0;
    0,      0,      0,      0,      2.5*10^(-4)
    ];

data.Pm = [
    1,      0,      0,      0,      0;
    0,      0.013,  0,      0,      0;
    0,      0,      pi^2,   0,      0;
    0,      0,      0,      1,      0;
    0,      0,      0,      0,      2.5*10^(-4)
    ];

x0 = [0; 0; 0; 0; 0]; % Initial values for the discrete states, modify
data.xm = [0; 0; 0; 0; 0];





%% Kalman filter update
%(1) Kalman gain
L = data.Pm*data.C'*inv(data.C*data.Pm*data.C'+data.R);
%(2) Update estimate
xh = data.xm + L*(u(2)-data.C*data.xm);
%(3) Update covariance
data.P = (eye(5)-L*data.C)*data.Pm*(eye(5)-L*data.C)' + L*data.R*L';
%(4) Project ahead
data.xm = data.A*xh + data.B*u(1);
data.Pm = data.A*data.P*data.A'+data.Q;
sys=data.xm;

