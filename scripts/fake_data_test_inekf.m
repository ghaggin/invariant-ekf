clear; close all; format compact;
set(0, 'DefaultTextInterpreter', 'latex');
set(0, 'DefaultLegendInterpreter', 'latex');
set(0, 'DefaultAxesFontSize', 12)

% Changing the number of random samples will break this seed
% make sure to regenerate everything with a new seed if the
% number of samples generated in the script changes
rng(2)
addpath('filters');
addpath('helper');

%--------------------------------------------------------------
% Fake data time limits and resolution
% lower resolution with noise free data
% should cause the prediction to improve.
time.tmin = 0;
time.tmax = 2;
time.dt = 1e-3;
%--------------------------------------------------------------

%--------------------------------------------------------------
% Covariance for sensor noise
% set noise.add_noise = false 
% to have perfect noise free data.
% If add_noise if false, accel_noise and 
% gyro_noise can be anything.
%
% Q1 and Q2 are two random matrices
% to try to create really bad noise
% with a ton of cross corelation between
% states
noise.add_noise = true;
% m = 100;
Q1 = randn(3,3); % dont comment out so seeding works 
Q2 = randn(3,3); % dont comment out so seeding works
% noise.accel_noise = m*Q1*Q1';
% noise.gyro_noise = m*Q2*Q2';
noise.accel_noise = ((6*9.81)^2)*eye(3);
noise.gyro_noise = (1^2)*eye(3);
%--------------------------------------------------------------

%--------------------------------------------------------------
% Set the frequency of the correction step (Hz)
%  - Increase the frequency to test robustness of filter
%    to slow updates
f_cor = 10;
dt_cor = 1/f_cor;
%--------------------------------------------------------------

%--------------------------------------------------------------
% Generate the fake data
% see function definition
% for input and output variable
% definitions.
[omega, accel, ~, ~, gt, init, wf_data] = gen_fake_data(time, noise);
%--------------------------------------------------------------

%--------------------------------------------------------------
% Position
% Sig_p0 = (25^2)*eye(3);
% L = chol(Sig_p0, 'lower');
% p0 = init.p0  + L*randn(3,1)
p0 = init.p0;

% Velocity
% Sig_v0 = (5^2)*eye(3);
% L = chol(Sig_p0, 'lower');
% v0 = init.v0  + L*randn(3,1)
v0 = init.v0;

% Rotation
% Sig_R0 = (0.05^2)*eye(3);
% L = chol(Sig_R0, 'lower');
% delta = L*randn(3,1);
% R0 = init.R0 * expm(skew(delta))
R0 = init.R0;
%--------------------------------------------------------------

%--------------------------------------------------------------
% Initialize the filter (with initial condition)
% Note: the polynomial function created by 
% gen_fake_data almost definitely wont be zero
% at t = 0
ekf = LIEKF(R0, p0, v0)
%--------------------------------------------------------------

%--------------------------------------------------------------
% Book Keeping

% Set the time data from the accelerometer
t = accel.t;
N = length(t);

% Initialize the position solution
p_sol = zeros(3,N);
p_sol(:,1) = ekf.mu(1:3, 5);
pos = [gt.x;gt.y;gt.z];

% Initialize the theta solution to 
% visualize the rotation matrix
theta = zeros(3, N);
theta_sol = zeros(3, N);
theta(:,1) = Log(gt.R{1});
theta_sol(:,1) = Log(ekf.mu(1:3,1:3));
%--------------------------------------------------------------

%--------------------------------------------------------------
% Run the simulation on the data
t_cor = t(1);  %Time of first correction
for i = 1:N-1
    % Set dt off time data
    dt = t(i+1) - t(i);

    % Set the acceleration from the fake data
    a = [accel.x(i); accel.y(i); accel.z(i)];
    w = [omega.x(i); omega.y(i); omega.z(i)];
    
    % Run the ekf prediction step
    ekf.prediction(w, a, dt);
    
    % Run the ekf correction step
    if t(i) >= t_cor
        gps = [gt.x(i); gt.y(i); gt.z(i)];
        ekf.correction(gps)

        % Next correct at t + dt_cor
        t_cor = t(i) + dt_cor;
    end

    % Extract the state from the filter
    [R, p, v] = ekf.getState(); 

    % Save the outputs (for plotting)
    p_sol(:,i+1) = p;
    theta_sol(:,i+1) = Log(R);

    % Save theta from gt (saves extra loop)
    theta(:,i+1) = Log(gt.R{i});
end
%--------------------------------------------------------------

%--------------------------------------------------------------
% Plot position and theta data to visualize
% the operation of the filter
figure;
subplot(311)
hold('on')
plot(t, p_sol(1,:), 'r')
plot(t, pos(1,:), 'k--')
ylabel('x')
subplot(312)
hold('on')
plot(t, p_sol(2,:), 'r')
plot(t, pos(2,:), 'k--')
ylabel('y')
subplot(313)
hold('on')
plot(t, p_sol(3,:), 'r')
plot(t, pos(3,:), 'k--')
legend('Estimation', 'Ground Truth', 'Location', 'northeast')
xlabel('Time')
ylabel('z')
saveas(gcf, 'fake_data_pos.png', 'png')

figure;
subplot(311)
hold('on')
plot(t, theta_sol(1,:), 'r')
plot(t, theta(1,:), 'k--')
ylabel('$\theta_1$')
subplot(312)
hold('on')
plot(t, theta_sol(2,:), 'r')
plot(t, theta(2,:), 'k--')
ylabel('$\theta_2$')
subplot(313)
hold('on')
plot(t, theta_sol(3,:), 'r')
plot(t, theta(3,:), 'k--')
legend('Estimation', 'Ground Truth', 'Location', 'northeast')
xlabel('Time')
ylabel('$\theta_3$')
saveas(gcf, 'fake_data_ang.png', 'png')
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
