%% Some constant from previous tasks
T_sample = 0.1;
w_0 = 0.4895;
lambda = 0.0657;
K = 0.1744;
T = 87.1542;
K_w = 0.1999;

%% Contiuous model
A_cont = [ 
0,           1,                       0,   0,       0;
-w_0*w_0,    -2*lambda*w_0,   0,   0,       0;
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

%% Exact discrete model

% Using Van Loan's method to derive A, B and Q
AQ_exponent = [A_cont, E_cont*Q_cont*transp(E_cont);
    zeros(5,5), -transp(A_cont)];

AQ = expm((AQ_exponent .* T_sample));

AB_exponent = [A_cont, B_cont; zeros(1,6)];
AB = expm((AB_exponent .* T_sample));

A = AQ(1:5,1:5);
B = AB(1:5,6);
Q = AQ(1:5,6:10)*transp(AQ(1:5,1:5));

% C is the same as in continuous model
C = C_cont;

% R_cont was found using var() on the output from model with zero input.
R_cont = 6.079e-07;
% R is averaged R_cont
R = R_cont/T_sample;

Pm0 = [
    1,      0,      0,      0,      0;
    0,      0.013,  0,      0,      0;
    0,      0,      pi^2,   0,      0;
    0,      0,      0,      1,      0;
    0,      0,      0,      0,      2.5*10^(-4)
    ];
xh0 = [0 0 0 0 0];
xm0 = [0 0 0 0 0];


data = struct(  ...
    'A',        A,        ...
    'B',        B,        ...
    'C',        C,        ... 
    'Q',        Q,        ...
    'R',        R, ...
    'Pm0', Pm0, ...
    'xm0', xm0,  ...
    'xh0', xh0 );


%% Simulation with estimated Psi as measurement
simtime = 600;
sim('ship', simtime);

figure(50);
subplot(3,1,1);
hold off;
plot([0,600],[30,30]);
hold on;
plot(compass.time,compass.data);
plot(psi_est.time,psi_est.data);
hold off;
title('Compass heading');
xlabel('Time [seconds]');
ylabel('Compass [degrees]');
legend('Reference heading','Actual compass heading','Estimated compass heading');
grid on;

subplot(3,1,2);
plot(psi_w(1,1:6006),psi_w(2,1:6006));
hold on;
plot(psi_w_est.time,psi_w_est.data);
hold off;
title('Wave influence on compass heading');
xlabel('Time [seconds]');
ylabel('Wave influence [degrees]');
legend('Actual wave influence', 'Estimated wave influence');
grid on;

subplot(3,1,3);
plot(rudder_forward.time,rudder_forward.data);
hold on;
plot(bias_est.time,bias_est.data);
hold off;
title('Rudder Angle');
xlabel('Time [seconds]');
ylabel('Rudder angle [degrees]');
legend('Rudder angle','Estimated bias');
grid on;

