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

% Get first observation that happens after a prediction
obsid = 1;
while(t_gps(obsid) < t_x(1))
    obsid = obsid + 1;
end

pos = zeros(3,test_N);
% vel = zeros(3,test_N); % can save velocity/orientation as well
% rot = cell(1,test_N);
for i = 1:test_N
    if i == 1
        dt(i) = t_x(i);
    else
        dt(i) = t_x(i) - t_x(i - 1);
        
        %Assume gyro/IMU are basically synchronous
        filter.prediction(w(i),a(i),dt(i));
        if(i < test_N)
            if(t_gps(obsid) > t_x(i) && t_x(i+1) > t_gps(obsid))
                gps = [xyz_gps(obsid,1); xyz_gps(obsid,2); xyz_gps(obsid,3)];
                filter.correction(gps);
                obsid = obsid + 1;
            end
        end
        
        %TBA: need to change covariance lie2Cartesian
        pos(:,i) = filter.mu(1:3,5);
    end
end

figure;
hold on;
plot3(xyz_gps(:,1), xyz_gps(:,2), xyz_gps(:,3), 'DisplayName','GPS');
plot3(pos(1,:), pos(2,:), pos(3,:),'DisplayName','LIEKF');
legend;
view(3);
