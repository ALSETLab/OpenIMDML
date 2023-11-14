% Example 11.7 motor starting 
% based on Juan's system 
% 

close all
clear all
clc

% converting from Xss, Xsp, Top to Xs, Xr, Xm
% PSLF data
Xss = 3.2; Xsp = 0.15; Top = 1.0;
Rr = Xss/Top/377
Xm = sqrt(Xss*(Xss-Xsp))
Xs = Xss - Xm
Xr = Xs

% the data file is d1m1mot_st.m (1 machine, 1 motor, startup)

s_simu 

% commands for making plots 

set_font_size
jay = sqrt(-1);

figure, plot(t,slip), grid
xlabel('Time (sec)'), ylabel('Slip (pu)')

figure, plot(t,p_mot), grid
xlabel('Time (sec)'), ylabel('pmot (pu)')

figure, plot(t,q_mot), grid
xlabel('Time (sec)'), ylabel('qmot (pu)')

bus_v_mag = abs(bus_v);
figure, plot(t,bus_v_mag), grid
xlabel('Time (sec)'), ylabel('bus v mag (pu)')
legend('Bus 1','Bus 2','Bus 3','Bus 4')

