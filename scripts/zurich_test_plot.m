clear; close all;
addpath('filters');
addpath('helper');

% Load / process data
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

% -------------------------------------------------------------------------
% Initialize filter

skew = @(u) [0 -u(3) u(2);
        u(3) 0 -u(1);
        -u(2) u(1) 0];

ekf = EKF();
inekf = LIEKF();

% Get first observation that happens after a prediction
obsid = 1;
while(t_gps(obsid) < t_x(1))
    obsid = obsid + 1;
end

pos_ekf = zeros(3,test_N);
pos_inekf = zeros(3,test_N);

for i = 2:test_N
    if i == 1
        dt = t_x; 
    else
        dt = t_x(i) - t_x(i - 1);
        
        %Assume gyro/IMU are basically synchronous
        ekf.prediction(w(i,:)',a(i,:)',dt);
        inekf.prediction(w(i,:)',a(i,:)',dt);
        
        %Measurement update
        if(i < test_N)
            if(t_gps(obsid) > t_x(i) && t_x(i+1) > t_gps(obsid))
                gps = [xyz_gps(obsid,1); xyz_gps(obsid,2); xyz_gps(obsid,3)];
                ekf.correction(gps);
                inekf.correction(gps);
                obsid = min(obsid + 1, length(t_gps));
            end
        end
        
        %TBA: need to change covariance lie2Cartesian
        pos_ekf(:,i) = ekf.mu(1:3);
        pos_inekf(:,i) = inekf.mu(1:3,5);
        if(mod(i,1000)==0)
           fprintf('Iteration: %d/%d\n',i,test_N); 
        end
    end
end

meas_used = T_GPS <= t_x(end);

% load gt
[~, ~, ~, ~, ~, x_gt, ~, y_gt, ~, z_gt] = loadGroundTruthAGL();
x_gt = x_gt - x_gt(1); y_gt = y_gt - y_gt(1); z_gt = z_gt - z_gt(1);
t_gt = linspace(0,T_X(end),length(x_gt));

% -------------------------------------------------------------------------
% traj plot
figure('DefaultAxesFontSize',14)
hold on;
plot3(XYZ_GPS(:,1), XYZ_GPS(:,2), XYZ_GPS(:,3),'b','LineWidth', 2);
plot3(x_gt, y_gt, z_gt,'--k','LineWidth', 4);
plot3(pos_ekf(1,:), pos_ekf(2,:), pos_ekf(3,:),'g','LineWidth', 2);
plot3(pos_inekf(1,:), pos_inekf(2,:), pos_inekf(3,:),'r','LineWidth', 2);
legend('gps', 'gt', 'EKF', 'InEKF', 'location', 'southeast')
hold off;
axis equal;

figure('DefaultAxesFontSize',14)
hold on;
plot3(XYZ_GPS(:,1), XYZ_GPS(:,2), XYZ_GPS(:,3),'b','LineWidth', 2);
plot3(x_gt, y_gt, z_gt,'--k','LineWidth', 4);
plot3(pos_inekf(1,:), pos_inekf(2,:), pos_inekf(3,:),'r','LineWidth', 2);
legend('gps', 'gt', 'InEKF', 'location', 'southeast')
hold off;
axis equal;

% -------------------------------------------------------------------------
% axis plot
figure;
subplot(3,1,1);
hold on;
plot(t_gps, XYZ_GPS(meas_used,1), 'b', 'LineWidth', 1);
plot(t_gt, x_gt, 'k--', 'LineWidth', 2);
plot(T_X(1:test_N),  pos_ekf(1,:), 'g', 'LineWidth', 1);
plot(T_X(1:test_N),  pos_inekf(1,:), 'r', 'LineWidth', 1);
legend('X_{GPS}','X_{GT}','X_{EKF}', 'X_{InEKF}', 'Location', 'eastoutside');
axis([0,T_X(test_N),-200,200])
%
subplot(3,1,2);
hold on;
plot(t_gps, XYZ_GPS(meas_used,2), 'b', 'LineWidth', 1);
plot(t_gt, y_gt, 'k--', 'LineWidth', 2);
plot(T_X(1:test_N),  pos_ekf(2,:), 'g', 'LineWidth', 1);
plot(T_X(1:test_N),  pos_inekf(2,:), 'r', 'LineWidth', 1);
legend('Y_{GPS}','Y_{GT}','Y_{EKF}', 'Y_{InEKF}', 'Location', 'eastoutside');
axis([0,T_X(test_N),-250,350])
%
subplot(3,1,3);
hold on;
plot(t_gps, XYZ_GPS(meas_used,3), 'b', 'LineWidth', 1);
plot(t_gt, z_gt, 'k--', 'LineWidth', 2);
plot(T_X(1:test_N),  pos_ekf(3,:), 'g', 'LineWidth', 1);
plot(T_X(1:test_N),  pos_inekf(3,:), 'r', 'LineWidth', 1);
legend('Z_{GPS}','Z_{GT}','Z_{EKF}', 'Z_{InEKF}', 'Location', 'eastoutside');
axis([0,T_X(test_N),-30,60])


