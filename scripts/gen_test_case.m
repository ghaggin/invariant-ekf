clear; close all; format compact; format long;

f = LIEKF();

w = [0.3; 0.5; 0.7];
a = [0.11; 0.13; 0.17];
gps = [0;0;0];

t0 = 0;
t1 = 0.19; %imu1
t2 = 0.27; %imu2
t3 = 0.29;
t4 = 0.57; %imu3
t5 = 0.83;

% gps for lla = [1;2;3];
% gps_enu = [
%  222310.934000408
%  111189.333769153
% -4847.78640277497
% ];

dt1 = t1 - t0;
dt2 = t2 - t1;
dt3 = t4 - t2;

f.prediction(w, a, 0.19);
f.prediction(w, a, 0.27 - 0.19);
f.correction(gps);
f.prediction(w, a, 0.57 - 0.27);
f.correction(gps);

f.mu
% f.Sigma