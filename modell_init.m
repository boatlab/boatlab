%% K and T
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
K = double(a(2))
T = double(b(3))
K_n = double(c(2))
T_n = double(d(3))

%% Transfer functions
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

sim('ship',simTime); % !!! simulink model must contain step block


figure(1);
hold on;
step(h,simTime,'b');
title('Step response - Parameters without noise');
plot(y.time,y.data,'r');
legend('Estimated','ship.model')
hold off;


figure(2);
hold on;
step(h_n,simTime,'b');
title('Step response - Parameters with noise');
plot(y.time,y.data,'r');
legend('Estimated with noise','ship.model')
hold off;

%%
%%5.2 Wave Spectrum
[pxx, f] = pwelch( psi_w(2,:), 4096, 10);
%Scaling to rads
pxx = pxx .* 1/(2*pi);
f = f .* (2*pi);

%Plot spectrum
figure(11)
plot(f(1:125),pxx(1:125))
legend('Estimate for Power Spectral Density Function for Psi_waves')


