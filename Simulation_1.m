% Digital PI Controller Comparison QSPICE vs. MATLAB, Rodrigo Anjos
%%% Clear Memory
clc
clear all
%%close all
%%%

%%% Load Packages
pkg load control
%%%

%%% Read CSV Files
data = csvread("Simulation_1.csv");
qspice_sim1_T   = data(:,1);
qspice_sim1_y   = data(:,2);
qspice_sim1_yk  = data(:,3);
%%%

%%% Simulate Target Plant and Controller
s   = tf('s');    % s Domain variable definition
Ts  = (0.5);      % Sample Period
z   = tf('z',Ts); % z Domain Variable definition
Gp  = 1/(s+1);    % Definition of the target plant continuous-time system

Gz  = c2d(Gp,Ts,'zoh');   % Calculation of the discrete-time plant transfer function

K   = 3.187;
zc  = 0.900;
pc  = 1.000;

Gc  = K*(z-zc)/(z-pc);      % Calculation of the discrete-time PI digital controller
Tz  = feedback(Gc*Gz,1,-1); % Calculation of the discrete-time closed loop response
[matlab_sim1_Y, matlab_sim1_T, matlab_sim1_X] = step(Tz,30);
%%%

%%% Compare the results from the simulations
%% Plot Both responses side-by-side
figure(1)
clf
step(Tz,30);
hold on
plot(qspice_sim1_T,qspice_sim1_yk,"r--");
title("QSPICE vs MATLAB Comparison - Simulation #1 Absolute Values", "fontsize", 16);
grid on
axis([0 30 0 1.5]);
xlabel('Time (seconds)');
xticks(0:5:30);
ylabel('Closed Loop Step Response (NA)');
yticks(0:0.1:1.5);

axes('Position',[.20 .675 .20 .20]);
box on
step(Tz,30);
hold on
plot(qspice_sim1_T,qspice_sim1_yk,"r--");
title("");
grid on
axis([0 2.5 0.5 1.4]);
xlabel('');
xticks(0:0.5:2.5);
xticklabels({'0','0.5','1','1.5','2','2.5'});
ylabel('');
yticks(0.5:0.05:1.4);
yticklabels({});

%% Plot Error Figure
figure(2)
clf
%ErrArray = CompareArrays(matlab_sim1_T,matlab_sim1_Y,qspice_sim1_T,qspice_sim1_yk);
ErrArray = CompareArrays(1);
%plot(qspice_sim1_T,matlab_sim1_Y-qspice_sim1_yk)
title("QSPICE vs MATLAB Comparison - Simulation #1 Error");
grid on
axis([0 30 0 1.5]);
xlabel('Time (seconds)');
xticks(0:5:30);
ylabel('Closed Loop Step Response (NA)');
yticks(0:0.1:1.5);
%%text(0.5,1.3,'\Delta = 1.2241'); %% 1.2541 - 1.2296 = 0.0
%%text(0.5,1.175,'1.2296');
%%%




