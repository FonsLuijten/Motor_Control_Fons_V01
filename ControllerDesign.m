% addpath(genpath('Measurements'))
% data = readNPY('CurrentPlant.npy');
close all;

s = tf('s');
Ts = 1/20000;

[num,den] = pade(1.5*Ts,4);
Td = tf(num,den);
freq = logspace(1,4,1000);

%% Current control plant
close all
R = 13.33; %Ohm
L = 1/(db2mag(-30.65))/(2*pi*1000);
P_cur = 1/(R+L*s)*Td;

figure;
bode(P_cur,2*pi*freq)
title('Plant Current control')

%% Current control
Kp_cur = 30;
Ki_cur =(1+700*2*pi/s);
C_cur = Kp_cur*Ki_cur;

OL_cur = P_cur*C_cur;
CL_cur = OL_cur/(1+OL_cur);

freq = logspace(1,4,1000);

figure;
bode(C_cur,2*pi*freq)
title('Current controller')
figure;
bode(OL_cur,2*pi*freq)
title('Open loop Current control')
figure;
bode(CL_cur,2*pi*freq)
title('Closed loop Current control')

%% Parameters Cart Pendulum
% Identification experiment to determine parameters
MKt = 1/db2mag(-39.5)/(2*pi*10)^2;    % Mass and Kt of cart only
MKt63 = 1/db2mag(-43.4)/(2*pi*10)^2;  % Mass and Kt of cart + 63gram

% (M+0.063)/Kt = MKt63;
% M/Kt = MKt;
% MKt63/MKt = ((M+0.063))/Kt/(M/Kt) = (M+0.063)/M
% M+0.063-MKt63/MKt*M = 0 => M =-0.063/(1-MKt63/MKt)

Mcart =-0.063/(1-MKt63/MKt);%*1e6;
Kt = Mcart/MKt;

Mcart = Mcart-0.039+0.086;

%% Position control cart only
close all
P_pos = 1/(Mcart*s^2)*Td;

fbw = 50;

% LeadLag (gain@BW = 1)
Kll_phi = leadlag(fbw/3,fbw*4);
Gainll = crossover(Kll_phi,fbw);
Kll_phi = Gainll*Kll_phi;

% LowPass (gain@BW = 1)
klp_phi = lowpass2(fbw*6,0.7);
Gainlp = crossover(klp_phi,fbw);
klp_phi = Gainlp*klp_phi;

% Integrator (gain@BW = 1)
Ki_phi = (1+fbw/6*2*pi/s);
Gaini = crossover(Ki_phi,fbw);
Ki_phi = Gaini*Ki_phi;

C_pos = Kll_phi*Ki_phi*klp_phi;%*Ki_pos;
Kp_posc = crossover(P_pos*C_pos,fbw);

Kp_pos = 5000;
C_pos = Kp_pos*C_pos;

OL_pos = P_pos*C_pos;
CL_pos = OL_pos/(1+OL_pos);


freq = logspace(1,4,1000);
figure;
bode(P_pos,2*pi*freq)
figure;
bode(C_pos,2*pi*freq)
figure;
bode(OL_pos,2*pi*freq)
figure;
bode(CL_pos,2*pi*freq)
figure;
bode(1/(1+OL_pos),2*pi*freq)


%% Position control plant pendulum
close all
Mrod = 0.043;   % Rod mass
Mpm = 0;        % Point mass added to rod

lrod = 0.31;   % Length of rod
lpm = 0.31;    % Distance of point mass from cart

% Parameters
Mrodcm = Mrod+Mpm;
lrodcm = Mrod/Mrodcm*lrod/2+Mpm/Mrodcm*lpm;
Irodcm = 1/12*Mrodcm*lrod^2+Mrodcm*(lrodcm-lrod/2)^2+Mrodcm*(lrodcm-lpm)^2;
g = 9.81;
d = 0.1; % Friction of cart
dp = 0; % Friction of pendulum

% Inverted pendulum
den = Irodcm*(Mrodcm+Mcart)+Mrodcm*Mcart*lrodcm^2;
T = [1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];
A = [0 1 0 0;...
    0 -(Irodcm+Mrodcm*lrodcm^2)*d/den Mrodcm^2*g*lrodcm^2/den -dp*Mrodcm*lrodcm/den;...
    0 0 0 1;...
    0 -Mrodcm*lrodcm*d/den Mrodcm*g*lrodcm*(Mcart+Mrodcm)/den -dp*(Mcart+Mrodcm)/den];
B = [0 (Irodcm+Mrodcm*lrodcm^2)/den 0 Mrodcm*lrodcm/den]';
C = [1 0 0 0;0 0 1 0];
D = [0;0];
sys = ss(inv(T)*A*T,inv(T)*B,C,D);
% sys2 = ss(A,B,[1 0;0 -1]*C,D);

% Non-inverted pendulum
A2 = inv(T)*[0 1 0 0;...
    0 -(Irodcm+Mrodcm*lrodcm^2)*d/den Mrodcm^2*g*lrodcm^2/den dp*Mrodcm*lrodcm/den;...
    0 0 0 1;...
    0 Mrodcm*lrodcm*d/den -Mrodcm*g*lrodcm*(Mcart+Mrodcm)/den -dp*(Mcart+Mrodcm)/den]*T;
B2 = inv(T)*[0 (Irodcm+Mrodcm*lrodcm^2)/den 0 -Mrodcm*lrodcm/den]';
sysni = ss(A2,B2,C,D);

figure;
x0 = [0 0 pi/2 0]';
initial(sysni,x0)
ylim([-15 15]);

% Plotting
figure;
bode(sys)
hold on
bode(sysni)
legend('invpend F2u','pend F2u')
xlim([1 100])
% bode(sys2)

%% Position control cascade loops
close all

% Controller design using cascade loops feedback: 
sys2 = sys(2); % start tuning the angle loop
fbw = 10;

% LeadLag (gain@BW = 1)
Kll_phi = leadlag(fbw/3,fbw*4);
Gainll = crossover(Kll_phi,fbw);
Kll_phi = Gainll*Kll_phi;

% LowPass (gain@BW = 1)
klp_phi = lowpass2(fbw*6,0.7);
Gainlp = crossover(klp_phi,fbw);
klp_phi = Gainlp*klp_phi;

% Integrator (gain@BW = 1)
Ki_phi = (1+fbw/6*2*pi/s);
Gaini = crossover(Ki_phi,fbw);
Ki_phi = Gaini*Ki_phi;

C_phi = Kll_phi*klp_phi*Ki_phi;
Kp_phi = crossover(sys2*C_phi,fbw);
C_phi = Kp_phi*C_phi;

OL_phi = sys*C_phi;
CL_phi = (1+OL_phi(2))\OL_phi;

figure;
bode(sys2)
title('Plant phi')
figure;
bode(OL_phi(2))
title('Open Loop phi')

disp('Eigenvalues of closed phi loop');
eig(CL_phi.A)


sys1 = CL_phi(1); % start tuning the cart position loop
figure;
bode(sys1)
title('Closed Loop phi to pos')

fbw = 0.2;

% LeadLag (gain@BW = 1)
Kll_x = leadlag(fbw/3,fbw*4);
Gainll = crossover(Kll_x,fbw);
Kll_x = Gainll*Kll_x;

% LowPass (gain@BW = 1)
klp_x = lowpass2(fbw*6,0.7);
Gainlp = crossover(klp_x,fbw);
klp_x = Gainlp*klp_x;

% Integrator (gain@BW = 1)
Ki_x = (1+fbw/6*2*pi/s);
Gaini = crossover(Ki_x,fbw);
Ki_x = Gaini*Ki_x;

C_x = Kll_x*Ki_x;%klp_x;%*Ki_x;
Kp_x = crossover(sys1*C_x,fbw);
C_x = -Kp_x*C_x;

OL = CL_phi*C_x;
CL = (1+OL(1))\OL;

figure;
bode(OL(1))
title('Open Loop pos')

figure;
bode(OL)
title('Open Loop phi/pos')

eig(CL.A)
CL2 = minreal(CL);
eig(CL2.A)

figure;
step(CL2)



% x0 = [0 0 0.1 0]';
% Acl = A-B*K;
% sys2 = ss(Acl,[],C,[]);
% initial(sys2,x0)
% ylim([-15 15]);

% x0 = [0 0 0.1 0]';
% initial(CL,x0)
% ylim([-15 15]);











