%% Test LIEKF filter
clear; close all;
[T_X, omega, accel, accel_b, T_GPS, XYZ_GPS] = loadPoseGPS();
% Initialize filter
filter = LIEKF;

test_N = 10;
w = omega(1:test_N,:);
a = accel(1:test_N,:);
b_g = zeros(test_N,3);
b_a = accel_b(1:test_N,:);
t_x = T_X(1:test_N,:);
dt = zeros(test_N - 1);
t_gps = T_GPS(1:test_N,:);
xyz_gps = XYZ_GPS(1:test_N,:); 

for i = 1:test_N
    if i == 1
        dt(i) = t_x(i);
    else
        dt(i) = t_x(i) - t_x(i - 1);
        filter.prediction(w(i),a(i),dt(i));
        gps = [xyz_gps(i,1); xyz_gps(i,2); xyz_gps(i,3)];
        filter.correction(gps);

        %TBA: need to account for asynchrony(if; t_x record; t_gps record; etc)
        %TBA: need to change covariance lie2Cartesian

    end
end
p = filter.mu(1:3,5);

figure;
hold on;
plot3(xyz_gps(:,1), xyz_gps(:,2), xyz_gps(:,3), 'DisplayName','GPS');
plot3(p(1,:), p(2,:), p(3,:),'DisplayName','LIEKF');
legend;
view(3);
