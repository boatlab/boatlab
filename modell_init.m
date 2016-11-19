

%% Deriving K and T
T_s = sym('T');
K_s = sym('K');

w_1 = 0.005;
w_2 = 0.05;

a_1 = 31.97;
a_2 = 0.78;
a_1n = 32;
a_2n = 2.3;

eq1 = (w_1^4 * a_1^2 * T_s^2) + (a_1^2 * w_1^2) == K_s^2;
eq2 = (w_2^4 * a_2^2 * T_s^2) + (a_2^2 * w_2^2) == K_s^2;
eq1n = (w_1^4 * a_1^2 * T_s^2) + (a_1^2 * w_1^2) == K_s^2;
eq2n = (w_2^4 * a_2n^2 * T_s^2) + (a_2n^2 * w_2^2) == K_s^2;

[a,b] = solve(eq1,eq2);
[c,d] = solve(eq1n,eq2n);
K = double(a(2));
T = double(b(3));
K_n = double(c(2));
T_n = double(d(3));

%% Transfer functions with and without noise
num = [K/T];
den = [1,1/T,0];
%without noise
h = tf(num,den);

num = [K_n/T_n];
den = [1,1/T_n,0];
%with noise
h_n = tf(num,den);

%% Simulation and plotting of step response
simTime = 50;

sim('ship_step',simTime); % !!! simulink model must contain step block

figure(1);
hold on;
step(h,simTime,'b');
title('Step response - Parameters without noise');
plot(compass.time,compass.data,'r');
legend('Estimated','Ship model')
hold off;

figure(2);
hold on;
step(h_n,simTime,'b');
title('Step response - Parameters with noise');
plot(compass.time,compass.data,'r');
legend('Estimated with noise','Ship model')
hold off;


%% 5.2 Wave Spectrum
[pxx, f] = pwelch( psi_w(2,:), 4096, 10, 20000);
%Scaling to rads
pxx = pxx .* 1/(2*pi);
rad_s = f .*(2*pi);

plot_length = length(rad_s)/15;

%Plot spectrum
hold off;
figure(11)
plot(rad_s(1:plot_length),pxx(1:plot_length))
legend('Estimate for Power Spectral Density Function for Psi_{waves}');
hold on;

%% Curve fitting
% w_0 read from plot
w_0 = 0.4895;
sigma = sqrt(9.652);

psiPSD = @(lambda,data)( 2*lambda*w_0*sigma * ( data ./ (sqrt( (w_0^2 - (data.^2)).^2 + ((2*lambda*w_0).*data).^2) ) ) ).^2;

lambda_fit = lsqcurvefit(psiPSD,0,rad_s,pxx);

K_w = 2*lambda_fit*w_0*sigma;

analytic_psi = psiPSD(lambda_fit,rad_s);

plot(rad_s(1:plot_length),analytic_psi(1:plot_length));
title('PSD Spectrums')
legend('Simulated PSD Spectrum','Analytic PSD Spectrum')
xlabel('Radians per second')
ylabel('Deg^2 per radian')
hold off;

%% 5.3 Controller Design
%Calculate parameters for PD-regulator
W_c = 0.1;
T_f = 1/(tan(-130*pi/180)*W_c);
T_d = T;
K_pd = sqrt(W_c^4 * T_f^2 + W_c^2)/K;

% Setup regulator transfer function
t_pd = [K_pd*T_d K_pd];
n_pd = [T_f 1];

h_pd = tf(t_pd, n_pd);

h_0 = h * h_pd;

% Check margins of open loop system
[gain_margin, phase_margin] = margin(h_0);
figure(20);
margin(h_0);
title_str = sprintf('Bode plot with margins of Open Loop system\n Gain margin: %f, Phase margin: %f, Wc = 0.1', gain_margin, phase_margin);
title(title_str);

[h_pd_n, h_pd_d] = tfdata(h_pd);
w_r = 30;

%% Simulate with step in reference
sim_time = 500;
sim('ship_pd', sim_time);

ref_line = w_r* ones(1,length(compass.time));

%Plot responses
figure(21);
hold off;
plot(compass.time, compass.data);
hold on;
plot(compass.time, ref_line);
grid on;
hold off;
str_title = sprintf('Plot of compass response with PD-regulator\n Noise on');
title(str_title);
xlabel('Time in seconds');
ylabel('Compass degrees');

figure(22);
plot(rudder.time, rudder.data);
grid on;
str_title = sprintf('Plot of rudder response with PD-regulator\n Noise on');
title(str_title);
xlabel('Time in seconds');
ylabel('Rudder degrees');
