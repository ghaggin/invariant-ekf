%% Test LIEKF filter
clear; close all;

[T_X, omega, accel, accel_b, T_GPS, XYZ_GPS] = loadPoseGPS();
test_N = length(omega); % Sets the number of IMU readings

w = omega(1:test_N,:);
a = accel(1:test_N,:);
b_g = zeros(test_N,3);
b_a = accel_b(1:test_N,:);
t_x = T_X(1:test_N,:);

meas_used = T_GPS <= t_x(end);
t_gps = T_GPS(meas_used,:);
xyz_gps = XYZ_GPS(meas_used,:); 

% Initialize filter
filter = EKF();
skew = @(u) [0 -u(3) u(2);
        u(3) 0 -u(1);
        -u(2) u(1) 0];

pos = zeros(3,test_N);

dt = t_x(1);
filter.prediction(rotm2eul(expm(skew(w(1,:)'))),a(1,:)',dt);
pos(:,1) = filter.mu(1:3);

x_ctr = 2;
gps_ctr = 1;
next_gps_time = t_gps(gps_ctr);

while x_ctr < length(t_x)
    if t_x(x_ctr) < next_gps_time
        dt = t_x(x_ctr) - t_x(x_ctr - 1);
        filter.prediction(rotm2eul(expm(skew(w(x_ctr,:)'))),a(x_ctr,:)',dt);
        x_ctr = x_ctr + 1;
        pos(:,x_ctr) = filter.mu(1:3);
    elseif gps_ctr < length(t_gps)
        gps = [xyz_gps(gps_ctr,1); xyz_gps(gps_ctr,2); xyz_gps(gps_ctr,3)];
        filter.correction(gps);
        gps_ctr = gps_ctr + 1;
        next_gps_time = t_gps(gps_ctr);
    else
        next_gps_time = inf;
    end
end

loadGroundTruthAGL
x_gt = x_gt - x_gt(1); y_gt = y_gt - y_gt(1); z_gt = z_gt - z_gt(1);
t_gt = linspace(0,T_X(end),length(x_gt));

figure;
hold on;
plot3(XYZ_GPS(:,1), XYZ_GPS(:,2), XYZ_GPS(:,3),'-g','DisplayName','GPS');
plot3(x_gt, y_gt, z_gt,'-k','DisplayName','EKF');
plot3(pos(1,:), pos(2,:), pos(3,:),'-r','DisplayName','EKF');
hold off;
axis equal;
legend;



figure;
subplot(3,1,1);
plot(T_X(1:test_N),  pos(1,:)); hold on;
plot(t_gps, XYZ_GPS(meas_used,1));
plot(t_gt, x_gt);
legend('X_{est}','X_{GPS}','X_{GT}');
axis([0,T_X(test_N),-200,200])

subplot(3,1,2);
plot(T_X(1:test_N),  pos(2,:)); hold on;
plot(t_gps, XYZ_GPS(meas_used,2));
plot(t_gt, y_gt);
legend('Y_{est}','Y_{GPS}','Y_{GT}');
axis([0,T_X(test_N),-250,350])

subplot(3,1,3);
plot(T_X(1:test_N),  pos(3,:)); hold on;
plot(t_gps, XYZ_GPS(meas_used,3));
plot(t_gt, z_gt);
legend('Z_{est}','Z_{GPS}','Z_{GT}');
axis([0,T_X(test_N),-30,60])



