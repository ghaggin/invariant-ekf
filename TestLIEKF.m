%% Test LIEKF filter
clear; close all;
[T_X, omega, accel, accel_b, T_GPS, XYZ_GPS] = loadPoseGPS();
% Initialize filter
filter = LIEKF;

test_N = 40000; %Sets the number of IMU readings
% test_N = length(T_X);

w = omega(1:test_N,:);
a = accel(1:test_N,:);
b_g = zeros(test_N,3);
b_a = accel_b(1:test_N,:);
t_x = T_X(1:test_N,:);
% w = omega;
% a = accel;
% b_g = zeros;
% b_a = accel_b;
% t_x = T_X;

% t_gps = T_GPS(1:test_N,:);
% xyz_gps = XYZ_GPS(1:test_N,:); 
t_gps = T_GPS;
xyz_gps = XYZ_GPS; 

p_sol = zeros(3,test_N);
theta_sol = zeros(3,test_N);
c_gps = 2;

for i = 1 : test_N-1
    
        dt = t_x(i+1) - t_x(i);
        filter.prediction(w(i,:)',a(i,:)',dt);
        
        if t_x(i) > t_gps(c_gps)
            gps = xyz_gps(c_gps,:)';
            filter.correction(gps);
            c_gps = c_gps + 1;
        end
        
        % Extract the state from the filter
        [R, p, v] = filter.getState(); 
        
        %Why we have R = NaN(3)???_
        % Save the outputs (for plotting)
        p_sol(:,i+1) = p;
        theta_sol(:,i+1) = Log(R);

end


% figure;
% plot3(XYZ_GPS(meas_used,1), XYZ_GPS(meas_used,2), XYZ_GPS(meas_used,3), 'DisplayName','GPS'); hold on;
% plot3(pos(1,:), pos(2,:), pos(3,:),'ro','DisplayName','LIEKF');
% legend;
% view(3);
figure;
plot3(p_sol(1,:), p_sol(2,:), p_sol(3,:),'DisplayName','LIEKF');
legend;
view(3);

%followings are the same as fake

% Plot position and theta data to visualize
% the operation of the filter
figure;
subplot(311)
hold('on')
plot(t_x, p_sol(1,:), 'r')
title("Position")
subplot(312)
hold('on')
plot(t_x, p_sol(2,:), 'r')
subplot(313)
hold('on')
plot(t_x, p_sol(3,:), 'r')

figure;
subplot(311)
hold('on')
plot(t_x, theta_sol(1,:), 'r')
title("Theta")
subplot(312)
hold('on')
plot(t_x, theta_sol(2,:), 'r')
subplot(313)
hold('on')
plot(t_x, theta_sol(3,:), 'r')
%--------------------------------------------------------------

%-------------------------------------------------------------
% Helper functions, mostly for SO3 stuff
function ux  = skew(u)
    ux = [
        0 -u(3) u(2)
        u(3) 0 -u(1)
        -u(2) u(1) 0
    ];
end

function u = unskew(ux)
    u(1,1) = -ux(2,3);
    u(2,1) = ux(1,3);
    u(3,1) = -ux(1,2);
end

function w = Log(R)
    w = unskew(logm(R));
end

function J = J_l(theta)
    t_x = skew(theta);
    t = norm(theta);

    J = eye(3) + (1 - cos(t))/t^2 * t_x + (t - sin(t))/t^3*(t_x)^2;
end
%-------------------------------------------------------------
