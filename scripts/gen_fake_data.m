clear; close all; format compact;

wf_data = generate_world_frame()

% wf_data = {t, pos, vel, acc, R, omega}
function wf_data = generate_world_frame()
    tmin = 0;
    tmax = 10;
    dt = 0.01;

    % polynomial parameters
    ord = 5;
    max_val = 20;

    % Generate time data
    t = tmin:dt:tmax;

    [pos.x, vel.x, acc.x] = gen_pva(ord, max_val, t);
    [pos.y, vel.y, acc.y] = gen_pva(ord, max_val, t);
    [pos.z, vel.z, acc.z] = gen_pva(ord, max_val, t);

    [R, omega] = gen_rot(ord, max_val, t);

    % Store data in output struct
    wf_data.t = t;
    wf_data.pos = [pos.x; pos.y; pos.z];
    wf_data.vel = [vel.x; vel.y; vel.z];
    wf_data.acc = [acc.x; acc.y; acc.z];
    wf_data.R = R;
    wf_data.omega = omega;
end

% Generate the rotational data
% R = cell of rotation matrices representing
%       vehicle orientation in the world frame
%       at time t
% omega = body frame angular rates om = Log(inv(R_{i})R_{i+1})/dt;
function [R, omega] = gen_rot(ord, max_val, t)
    % Extend t so that we have one rotation matrix past
    % the end
    dt = t(end) - t(end-1);
    t(end+1) = t(end) + dt;

    [tx_p, ~, ~] = make_poly(ord, max_val);
    [ty_p, ~, ~] = make_poly(ord, max_val);
    [tz_p, ~, ~] = make_poly(ord, max_val);
    t = [polyval(tx_p, t); polyval(ty_p, t); polyval(tz_p, t)];

    % Generate the rotation matrices
    % from the t data and store in cell
    R = cell(1, length(t));
    for i = 1:length(t)
        R{i} = angle_axis(t(:, i));
    end
    
    N = length(t) - 1;
    omega = zeros(3, N);
    for i = 1:N
        omega(:,i) = (1/dt) * Log(transpose(R{i})*R{i+1});
    end

    % Shorten R by one to agree with
    % omega and t
    R = R(1:N);
end

% Generate the pos, vel, acc (pva) for the world frame
function [pos, vel, acc] = gen_pva(ord, max_val, t)
    [p_p, v_p, a_p] = make_poly(ord, max_val);
    pos = polyval(p_p, t);
    vel = polyval(v_p, t);
    acc = polyval(a_p, t);
end

% Make a polynomial out of random
% values in [-max_val, max_val]
% also return first and second derivatives
function [p, v, a] = make_poly(ord, max_val)
    p = rand(ord, 1)*2*max_val - max_val;
    v = polyder(p);
    a = polyder(v);
end

% Construct rotation matrix from
% angle axis where axis u = t/norm(t)
% and angle theta = norm(t)
function R = angle_axis(t)
    % Extract theta and u from t
    theta = norm(t);
    u = t/theta;
    
    % Check that u is a collumn vector
    % so the outer product works below
    u = reshape(u, [3,1]);

    % Angle axis formula
    % https://en.wikipedia.org/wiki/Rotation_matrix#Rotation_matrix_from_axis_and_angle
    R = cos(theta)*eye(3) + sin(theta)*skew(u) + (1 - cos(theta))*(u * u');
end

% Skew operator (cross-product matrix, etc.)
function ux = skew(u)
    ux = [
            0  -u(3)   u(2)
         u(3)      0  -u(1)
        -u(2)   u(1)      0
    ];
end

function u = unskew(ux)
    u(3,1) = -ux(1,2);
    u(2,1) = ux(1,3);
    u(1,1) = -ux(2,3);
end

function w = Log(R)
    w = unskew(logm(R));
end