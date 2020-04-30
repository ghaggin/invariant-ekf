% Test LIEKF filter
clear; close all;
addpath('filters');
addpath('helper');

[T_X, omega, accel, accel_b, T_GPS, XYZ_GPS] = loadPoseGPS();

% rx = rotx(95.538828);
% ry = roty(-73.283629);
% rz = rotz(9.170787);
% R0 = rz*ry*rx;
% R0 = expm(wedge([95.538828	-73.283629	9.170787]*pi/180));
% v0 = [0;0;0];
% p0 = [0;0;0];

% Initialize filter
% filter = LIEKF(R0,v0,p0);
filter = LIEKF;
test_N = length(omega); %Sets the number of IMU readings
% test_N = 30000;

w = omega(1:test_N,:);
a = accel(1:test_N,:);
b_g = zeros(test_N,3);
b_a = accel_b(1:test_N,:);
t_x = T_X(1:test_N,:);
% dt = [0.02; t_x(2:test_N)-t_x(1:test_N-1)];

meas_used = T_GPS <= t_x(end);
t_gps = T_GPS(meas_used,:);
xyz_gps = XYZ_GPS(meas_used,:); 


% Get first observation that happens after a prediction
obsid = 1;
while(t_gps(obsid) < t_x(1))
    obsid = obsid + 1;
end

pos = zeros(3,test_N);
% vel = zeros(3,test_N); % can save velocity/orientation as well
% rot = cell(1,test_N);
for i = 2:test_N
    if i == 1
        dt(i) = t_x(i); 
    else
        dt(i) = t_x(i) - t_x(i - 1);
        
        %Assume gyro/IMU are basically synchronous
        filter.prediction(w(i,:)',a(i,:)',dt(i));
        
        %Measurement update
        if(i < test_N)
            if(t_gps(obsid) > t_x(i) && t_x(i+1) > t_gps(obsid))
                gps = [xyz_gps(obsid,1); xyz_gps(obsid,2); xyz_gps(obsid,3)];
                filter.correction(gps);
                obsid = min(obsid + 1, length(t_gps));
            end
        end
        
        %TBA: need to change covariance lie2Cartesian
        pos(:,i) = filter.mu(1:3,5);
        if(mod(i,1000)==0)
           fprintf('Iteration: %d/%d\n',i,test_N); 
        end
    end
end

meas_used = T_GPS <= t_x(end);

figure;
plot3(XYZ_GPS(meas_used,1), XYZ_GPS(meas_used,2), XYZ_GPS(meas_used,3), 'DisplayName','GPS'); hold on;
plot3(pos(1,:), pos(2,:), pos(3,:),'.r','DisplayName','LIEKF');
legend;
view(3);

[~, ~, ~, ~, ~, x_gt, ~, y_gt, ~, z_gt] = loadGroundTruthAGL();
x_gt = x_gt - x_gt(1); y_gt = y_gt - y_gt(1); z_gt = z_gt - z_gt(1);
t_gt = linspace(0,T_X(end),length(x_gt));

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
