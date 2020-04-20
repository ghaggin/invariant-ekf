clear; close all; format compact;

tmin = 0;
tmax = 1;
dt = 1e-3;

[omega, accel, ~, ~, gt, init, wf_data] = gen_fake_data(tmin, tmax, dt);

ekf = LIEKF(init.R0, init.p0, init.v0)
t = accel.t;
N = length(t);

p_sol(:,1) = init.p0;
pos = [gt.x;gt.y;gt.z];

theta(:,1) = Log(ekf.mu(1:3,1:3));
theta_sol(:,1) = Log(gt.R{1});


for i = 1:N-1
    dt = t(i+1) - t(i);

    a = [accel.x(i); accel.y(i); accel.z(i)];
    w = [omega.x(i); omega.y(i); omega.z(i)];

    ekf.prediction(w, a, dt);

    [R, p, v] = ekf.getState();

    p_sol(:,i+1) = p;

    theta_sol(:,i+1) = Log(R);
    theta(:,i+1) = Log(gt.R{i});
end

figure;
subplot(311)
hold('on')
plot(t, p_sol(1,:), 'r')
plot(t, pos(1,:), 'k--')
subplot(312)
hold('on')
plot(t, p_sol(2,:), 'r')
plot(t, pos(2,:), 'k--')
subplot(313)
hold('on')
plot(t, p_sol(3,:), 'r')
plot(t, pos(3,:), 'k--')

figure;
subplot(311)
hold('on')
plot(t, theta_sol(1,:), 'r')
plot(t, theta(1,:), 'k--')
subplot(312)
hold('on')
plot(t, theta_sol(2,:), 'r')
plot(t, theta(2,:), 'k--')
subplot(313)
hold('on')
plot(t, theta_sol(3,:), 'r')
plot(t, theta(3,:), 'k--')

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