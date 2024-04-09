% Digital PI Controller Comparison QSPICE vs. MATLAB, Rodrigo Anjos
%%% Clear Memory
clc
clear all
%%close all
%%%

%%% Load Packages
pkg load control
pkg load signal
%%%

%%% Read CSV Files
data = csvread("Simulation_1.csv");
qspice_sim1_T   = data(:,1);
qspice_sim1_y   = data(:,2);
qspice_sim1_yk  = data(:,3);
data = csvread("Simulation_2.csv");
qspice_sim2_T   = data(:,1);
qspice_sim2_yk  = data(:,2);
data = csvread("Simulation_3.csv");
qspice_sim3_T   = data(:,1);
qspice_sim3_y   = data(:,2);
qspice_sim3_yk  = data(:,3);
data = csvread("Simulation_4.csv");
qspice_sim4_T   = data(:,1);
qspice_sim4_yk  = data(:,2);
data = csvread("Simulation_5.csv");
qspice_sim5_T   = data(:,1);
qspice_sim5_yk  = data(:,2);
qspice_sim5_uk  = data(:,3);
%%data = csvread("Simulation_6.csv");
%%qspice_sim6_T   = data(:,1);
%%qspice_sim6_yk  = data(:,2);
data = csvread("Simulation_7.csv");
qspice_sim7_T   = data(:,1);
qspice_sim7_yk  = data(:,2);
data = csvread("Simulation_8.csv");
qspice_sim8_T   = data(:,1);
qspice_sim8_yk  = data(:,2);
qspice_sim8_uk  = data(:,3);
data = csvread("Simulation_9.csv");
qspice_sim9_T   = data(:,1);
qspice_sim9_yk  = data(:,2);
data = csvread("Simulation_10.csv");
qspice_sim10_T   = data(:,1);
qspice_sim10_yk  = data(:,2);
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
title("QSPICE vs MATLAB Comparison - Simulation #1 Absolute Values", "fontsize", 16,'FontWeight','Bold');
grid on
axis([0 30 0 1.5]);
xlabel('Time (seconds)');
xticks(0:5:30);
ylabel('Closed Loop Step Response');
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
xticklabels({});
ylabel('');
yticks(0.5:0.05:1.4);
yticklabels({});
saveas(gcf,'./figs/Simulation_1_Results.png');


%% Plot Error Figure
figure(2)
clf
hold on
plot(qspice_sim2_T(0001:1002),qspice_sim2_yk(0001:1002),'r--');
plot(qspice_sim2_T(1003:2003),qspice_sim2_yk(1003:2003),'k-');
plot(qspice_sim2_T(2004:3004),qspice_sim2_yk(2004:3004),'g');
plot(qspice_sim2_T(3005:4005),qspice_sim2_yk(3005:4005),'b--');
title("QSPICE vs MATLAB Comparison - Simulation #2 Simulation Stability", "fontsize", 16,'FontWeight','Bold');
grid on
axis([0 30 0 2]);
xlabel('Time (seconds)');
xticks(0:5:30);
ylabel('Closed Loop Step Response');
yticks(0:0.2:2);
legend('\Delta t=1n','\Delta t=1u','\Delta t=1m','\Delta t=1');
saveas(gcf,'./figs/Simulation_2_Results.png');

%% Plot Both responses side-by-side
figure(3)
clf
step(Tz,30,1,"k-");
hold on
plot(qspice_sim3_T,qspice_sim3_yk ,"b--");
%plot(qspice_sim3_T,qspice_sim3_y  ,"b-");

title("QSPICE vs MATLAB Comparison - Simulation #3 Absolute Values", "fontsize", 16,'FontWeight','Bold');
grid on
axis([0 30 0 1.5]);
xlabel('Time (seconds)');
xticks(0:5:30);
ylabel('Closed Loop Step Response');
yticks(0:0.1:1.5);
saveas(gcf,'./figs/Simulation_3_Results.png');

%%
figure(4)
clf
%
medianWindow = 10;
%YInterpMATLAB = interp1(matlab_sim1_T, matlab_sim1_Y, qspice_sim1_T,'previous');
Yerr_sim1_unfilt = abs(interp1(matlab_sim1_T, matlab_sim1_Y, qspice_sim1_T,'previous')-qspice_sim1_yk);
Yerr_sim3_unfilt = abs(interp1(matlab_sim1_T, matlab_sim1_Y, qspice_sim3_T,'previous')-qspice_sim3_yk);
Yerr_sim1_filt = abs(medfilt1(Yerr_sim1_unfilt,medianWindow));
Yerr_sim3_filt = abs(medfilt1(Yerr_sim3_unfilt,medianWindow));
%
subplot(2,1,1);
hold on
ylim([1e-9 1e0]);
semilogy(qspice_sim1_T, Yerr_sim1_unfilt);
semilogy(qspice_sim3_T, Yerr_sim3_unfilt);
grid on
%
title('Unfiltered Error','fontsize',12);
legend(...
        strcat('Block Diagram Controller RMS Error = ',num2str(1000*rms(Yerr_sim1_unfilt),"%5.2f"),' mV_{rms}'),...
        strcat(   'Integrated Controller RMS Error = ',num2str(1000*rms(Yerr_sim3_unfilt),"%5.2f"),' mV_{rms}'),...
        'fontsize', 12 ...
      );
%
subplot(2,1,2);
hold on
ylim([1e-9 1e0]);
semilogy(qspice_sim1_T, Yerr_sim1_filt);
semilogy(qspice_sim3_T, Yerr_sim3_filt);
grid on
title(strcat('Filtered Error (Moving Median Window=',num2str(medianWindow),')'),'fontsize',12);
legend(...
        strcat('Block Diagram Controller RMS Error = ',num2str(1000*rms(Yerr_sim1_filt),"%5.2f"),' mV_{rms}'),...
        strcat(   'Integrated Controller RMS Error = ',num2str(1000000*rms(Yerr_sim3_filt),"%5.2f"),' uV_{rms}'),...
        'fontsize', 12 ...
      );

S = axes('visible','off','title',"QSPICE vs MATLAB Comparison - Simulation #4 Error Identification", "fontsize", 16,'FontWeight','Bold');
saveas(gcf,'./figs/Simulation_4_Errors.png');

%%
figure(5)
clf
Yerr_sim4_1_filt = medfilt1(abs(interp1(matlab_sim1_T, matlab_sim1_Y, qspice_sim4_T(0001:1001),'previous')-qspice_sim4_yk(0001:1001)),medianWindow);
Yerr_sim4_2_filt = medfilt1(abs(interp1(matlab_sim1_T, matlab_sim1_Y, qspice_sim4_T(1002:2002),'previous')-qspice_sim4_yk(1002:2002)),medianWindow);
Yerr_sim4_3_filt = medfilt1(abs(interp1(matlab_sim1_T, matlab_sim1_Y, qspice_sim4_T(2003:3003),'previous')-qspice_sim4_yk(2003:3003)),medianWindow);
Yerr_sim4_4_filt = medfilt1(abs(interp1(matlab_sim1_T, matlab_sim1_Y, qspice_sim4_T(3004:4004),'previous')-qspice_sim4_yk(3004:4004)),medianWindow);
Yerr_sim4_5_filt = medfilt1(abs(interp1(matlab_sim1_T, matlab_sim1_Y, qspice_sim4_T(4005:5005),'previous')-qspice_sim4_yk(4005:5005)),medianWindow);
Yerr_sim4_6_filt = medfilt1(abs(interp1(matlab_sim1_T, matlab_sim1_Y, qspice_sim4_T(5006:6006),'previous')-qspice_sim4_yk(5006:6006)),medianWindow);
%
subplot(2,1,1);
hold on
ylim([1e-9 1e0]);
semilogy(qspice_sim4_T(0001:1001), Yerr_sim4_1_filt,'y-');
semilogy(qspice_sim4_T(1002:2002), Yerr_sim4_2_filt,'c-');
semilogy(qspice_sim4_T(2003:3003), Yerr_sim4_3_filt,'b-');
semilogy(qspice_sim4_T(3004:4004), Yerr_sim4_4_filt,'g-');
semilogy(qspice_sim4_T(4005:5005), Yerr_sim4_5_filt,'r-');
semilogy(qspice_sim4_T(5006:6006), Yerr_sim4_6_filt,'k-');
grid on
%
title('QSPICE/MATLAB Numerical Error vs. \Deltat_{CPU}','fontsize',12);
legend(...
        "\Deltat_{CPU}=1m",...
        "\Deltat_{CPU}=1u",...
        "\Deltat_{CPU}=1n",...
        "\Deltat_{CPU}=1p",...
        "\Deltat_{CPU}=1f",...
        "\Deltat_{CPU}=0",...
        'fontsize', 12 ...
      );
%
subplot(2,1,2);
hold on
xlim([1e-18 1e-3]);
ERR_RMS_X = [1e-18,1e-15,1e-12,1e-9,1e-6,1e-3];
ERR_RMS_Y = [...
              rms(Yerr_sim4_6_filt),...
              rms(Yerr_sim4_5_filt),...
              rms(Yerr_sim4_4_filt),...
              rms(Yerr_sim4_3_filt),...
              rms(Yerr_sim4_2_filt),...
              rms(Yerr_sim4_1_filt),...
            ];
semilogx(ERR_RMS_X, ERR_RMS_Y,'ko-');
grid on
title('RMS Error vs. \Deltat_{CPU}');

S = axes('visible','off','title',"QSPICE vs MATLAB Comparison - Simulation #5 RMS Error Quantification", "fontsize", 16,'FontWeight','Bold');
saveas(gcf,'./figs/Simulation_5_ErrorRMS.png');

%%
figure(6)
clf
subplot(2,1,1);
hold on
plot(qspice_sim5_T(0001:1002),qspice_sim5_yk(0001:1002),'r-');
plot(qspice_sim5_T(1003:2003),qspice_sim5_yk(1003:2003),'k--');
grid on
title('Controller Input Signal y(kT)','fontsize',12);
legend(...
        'Unlimited Range: V_{CC}=\pm 100V',...
        'Limited Range: V_{CC}=\pm 12V',...
        'fontsize', 12 ...
      );

subplot(2,1,2);
hold on
plot(qspice_sim5_T(0001:1002),qspice_sim5_uk(0001:1002),'r-');
plot(qspice_sim5_T(1003:2003),qspice_sim5_uk(1003:2003),'k--');
grid on
title('Controller Output Signal u(kT)','fontsize',12);
legend(...
        'Unlimited Range: V_{CC}=\pm 100V',...
        'Limited Range: V_{CC}=\pm 12V',...
        'fontsize', 12 ...
      );

S = axes('visible','off','title',"QSPICE Simulation #5 ADC/DAC Limited Operational Range", "fontsize", 16,'FontWeight','Bold');
saveas(gcf,'./figs/Simulation_5_ADC_Lim_Op_Range.png');

%%
figure(7)
clf

S = axes('visible','off','title',"QSPICE Simulation #6 ADC/DAC Limited Range & Stability", "fontsize", 16,'FontWeight','Bold');
saveas(gcf,'./figs/Simulation_6_ADC_Lim_Op_Range_Stability.png');
%%
figure(8)
clf
hold on
plot(qspice_sim7_T(0001:1002),qspice_sim7_yk(0001:1002),'r-');
plot(qspice_sim7_T(1003:2003),qspice_sim7_yk(1003:2003),'g-');
plot(qspice_sim7_T(2004:3004),qspice_sim7_yk(2004:3004),'b-');
plot(qspice_sim7_T(3005:4005),qspice_sim7_yk(3005:4005),'k--');
grid on
legend(...
        'Bitdepth=08 V_{CC}/V_{EE}=\pm 12V LSB=',...
        'Bitdepth=10 V_{CC}/V_{EE}=\pm 12V LSB=',...
        'Bitdepth=12 V_{CC}/V_{EE}=\pm 12V LSB=',...
        'Bitdepth=16 V_{CC}/V_{EE}=\pm 12V LSB=',...
        'fontsize', 12 ...
      );

S = axes('visible','off','title',"QSPICE Simulation #7 ADC/DAC Quantization Error", "fontsize", 16,'FontWeight','Bold');
saveas(gcf,'./figs/Simulation_7_ADC_Quant_Err.png');

%%
figure(9)
clf
subplot(2,1,1);
hold on
plot(qspice_sim8_T(0001:1002),qspice_sim8_yk(0001:1002),'r-');
plot(qspice_sim8_T(1003:2003),qspice_sim8_yk(1003:2003),'g-');
plot(qspice_sim8_T(2004:3004),qspice_sim8_yk(2004:3004),'b-');
plot(qspice_sim8_T(3005:4005),qspice_sim8_yk(3005:4005),'k--');
grid on
title('Controller Input Signal y(kT)','fontsize',12);
legend(...
        'V_{EE} Noise Level= 1mV',...
        'V_{EE} Noise Level= 10mV',...
        'V_{EE} Noise Level= 100mV',...
        'V_{EE} Noise Level= 1V',...
        'fontsize', 12 ...
      );

subplot(2,1,2);
hold on
plot(qspice_sim8_T(0001:1002),qspice_sim8_uk(0001:1002),'r-');
plot(qspice_sim8_T(1003:2003),qspice_sim8_uk(1003:2003),'g-');
plot(qspice_sim8_T(2004:3004),qspice_sim8_uk(2004:3004),'b-');
plot(qspice_sim8_T(3005:4005),qspice_sim8_uk(3005:4005),'k-');
grid on
title('Controller Output Signal u(kT)','fontsize',12);
legend(...
        'V_{EE} Noise Level= 1mV',...
        'V_{EE} Noise Level= 10mV',...
        'V_{EE} Noise Level= 100mV',...
        'V_{EE} Noise Level= 1V',...
        'fontsize', 12 ...
      );
S = axes('visible','off','title',"QSPICE Simulation #8 Power Supply Noise Error", "fontsize", 16,'FontWeight','Bold');
saveas(gcf,'./figs/Simulation_8_PSU_Noise.png');

%%
figure(10)
clf
hold on
plot(qspice_sim9_T(0001:1002),qspice_sim9_yk(0001:1002),'r-');
plot(qspice_sim9_T(1003:2003),qspice_sim9_yk(1003:2003),'b-');
plot(qspice_sim9_T(2004:3004),qspice_sim9_yk(2004:3004),'k-');
plot(qspice_sim9_T(3005:4005),qspice_sim9_yk(3005:4005),'b--');
plot(qspice_sim9_T(4006:5006),qspice_sim9_yk(4006:5006),'r--');
grid on
legend(...
        '-50% Relative Shift',...
        '-10% Relative Shift',...
        '  0% Relative Shift',...
        '+10% Relative Shift',...
        '+50% Relative Shift',...
        'fontsize', 12 ...
      );

S = axes('visible','off','title',"QSPICE Simulation #9 Clock Frequency Shift Error", "fontsize", 16,'FontWeight','Bold');
saveas(gcf,'./figs/Simulation_9_CLK_Noise_Err.png');

%%
figure(11)
clf
hold on
plot(qspice_sim10_T(0001:1002),qspice_sim10_yk(0001:1002),'r-');
plot(qspice_sim10_T(1003:2003),qspice_sim10_yk(1003:2003),'g-');
plot(qspice_sim10_T(2004:3004),qspice_sim10_yk(2004:3004),'b-');
plot(qspice_sim10_T(3005:4005),qspice_sim10_yk(3005:4005),'k-');
plot(qspice_sim10_T(4006:5006),qspice_sim10_yk(4006:5006),'y-');
grid on
legend(...
        '01% Phase Jitter',...
        '05% Phase Jitter',...
        '10% Phase Jitter',...
        '25% Phase Jitter',...
        '50% Phase Jitter',...
        'fontsize', 12 ...
      );

S = axes('visible','off','title',"QSPICE Simulation #10 Clock Phase Jitter", "fontsize", 16,'FontWeight','Bold');
saveas(gcf,'./figs/Simulation_10_CLK_Phase_Jitter.png');

%%%




