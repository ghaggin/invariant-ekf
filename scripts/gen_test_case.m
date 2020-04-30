clear; close all; format compact; format long;

f = LIEKF();

w = [0.3; 0.5; 0.7];
a = [0.11; 0.13; 0.17];
gps = [0;0;0];

f.correction(gps);
f.prediction(w, a, 0.27);

f.mu