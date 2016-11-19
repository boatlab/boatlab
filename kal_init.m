T_sample = 0.1;
w_0 = 0.4895;
lambda = 0.0657;
K = 0.1744;
T = 87.1542;
K_w = 0.1999;

% Contiuous model
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
% Discrete system

% Using Van Loan's method to derive A, B and Q
AQ_exponent = [A_cont, E_cont*Q_cont*transp(E_cont); zeros(5,5), -transp(A_cont)];
AQ = expm((AQ_exponent .* T_sample));

AB_exponent = [A_cont, B_cont; zeros(1,6)];
AB = expm((AB_exponent .* T_sample));

A = AQ(1:5,1:5);
B = AB(1:5,6);
Q = AQ(1:5,6:10)*transp(AQ(1:5,1:5));

% C is the same
C = C_cont;

% R_cont is found using var() on the output from model with zero input.
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

simtime = 600;

timeline = 0:0.1:simtime;
compass_reference = 30;
referenceline = ones(length(timeline)) * compass_reference;

sim('ship', simtime);

figure(50);

subplot(2,1,1);
grid on;
plot(timeline,referenceline, 'b', compass.time,compass.data, 'r', psi_est.time,psi_est.data, 'c');
title('Compass heading');
xlabel('Time [seconds]');
ylabel('Compass [degrees]');
legend('Reference heading','Actual compass heading', 'Estimated compass heading without waves');

%{
subplot(2,1,2);
grid on;
plot(timeline, waveline);
hold on;
plot(psi_w_est.time,psi_w_est.data);
hold off;
title('Wave influence on compass heading');
xaxis('Time [seconds]');
yaxis('Wave influence [degrees]');
legendNames = ['Actual wave influence', 'Estimated wave influence'];
legend(legendNames);
%}

subplot(2,1,2);     
grid on;
plot(rudder.time,rudder.data);
hold on;
plot(bias_est.time,bias_est.data);
plot(rudder_forward.time,rudder_forward.data);

title('');
xlabel('Time [seconds]');
ylabel('Rudder angle [degrees]');
legend('Rudder angle before feed forward','Estimated bias','Rudder angle after feed forward');





