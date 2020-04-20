% Generates a fake data set to use for testing and analysis.
% Right now the function make perfect data based on grouth
% truth positions that are polynomial functions of time.
%
% TODO:
%   - Return data form (same as zurich?)
%   - Noise
%   - Biases
%   - Assynchronous measurements
function [omega, accel, gps, sig, gt, init, wf_data] = gen_fake_data(time, noise)
    %----------------------------------------------
    % Set time data
    tmin = time.tmin;
    tmax = time.tmax;
    dt = time.dt;
    t = tmin:dt:tmax;
    %----------------------------------------------

    %----------------------------------------------
    % Get the world frame data, see 
    % comment above function for details
    wf_data = generate_world_frame(t);
    %----------------------------------------------

    %----------------------------------------------
    % Make the body frame data
    %
    % Define an east-north-up coordinate system
    % with an origin at lla cordinates.
    % ENU -> x = east, y = north, z = up
    origin = [0;0;0];

    % Acceration measured by the acclerometer
    % due to local gravity.  The acclerometer
    % can't tell the difference between experiencing
    % an upward acceleration vs. a gravitational field
    % pulling it down (picture a mass suspended by springs)
    g = [0;0;-9.81];

    % Acceleration as measured by the accelerometer is
    % the world frame acceleration plus gravity accel
    % rotated into the body frame by R
    %
    % Noise is then added to this "true" body frame accel
    accel = zeros(3, size(wf_data.acc, 2));
    for i = 1:size(accel, 2)
        accel(:,i) = wf_data.R{i}' * (wf_data.acc(:,i) - g);
        if noise.add_noise
            Q_a = noise.accel_noise;
            L_a = chol(Q_a, 'lower');
            accel(:,i) = accel(:,i) + L_a*randn(3,1);
        end
    end

    % Set the measured rates to the generated 
    % angular rates
    omega = wf_data.omega;
    if noise.add_noise
        Q_w = noise.gyro_noise;
        L_w = chol(Q_w, 'lower');
        omega = omega + L_w*randn(3,size(omega, 2));
    end

    % Set the ground truth as the generated position
    gt.pos = wf_data.pos;
    gt.R = wf_data.R;

    % Convert ENU coordinates to GPS coordinates
    E = wgs84Ellipsoid();
    gps = zeros(3, size(wf_data.acc, 2));
    for i = 1:size(gps, 2)
        [lat, long, alt] = enu2geodetic(gt.pos(1,i), gt.pos(2,i), gt.pos(3,i), origin(1), origin(2), origin(3), E);
        gps(:,i) = [lat;long;alt];
    end

    sig =[ 
        4.6778    1.9437    0.0858
        1.9437   11.5621    5.8445
        0.0858    5.8445   22.4051
    ];
    [omega, accel, gps, sig, gt] = convertToZurich(t, omega, accel, gps, sig, gt);

    init.R0 = wf_data.R{1};
    init.p0 = wf_data.pos(:,1);
    init.v0 = wf_data.vel(:,1);
end

% Convert my form to the zurich form
function [omega_r, accel_r, gps_r, sig_r, gt_r] = convertToZurich(t, omega, accel, gps, sig, gt)
    omega_r.x = omega(1,:);
    omega_r.y = omega(2,:);
    omega_r.z = omega(3,:);
    omega_r.t = t ;

    accel_r.x = accel(1,:);
    accel_r.y = accel(2,:);
    accel_r.z = accel(3,:);
    accel_r.t = t;

    gps_r.lat = gps(1,:);
    gps_r.long = gps(2,:);
    gps_r.alt = gps(3,:);
    gps_r.t = t;

    sig_r = sig;

    gt_r.x = gt.pos(1,:);
    gt_r.y = gt.pos(2,:);
    gt_r.z = gt.pos(3,:);
    gt_r.R = gt.R;
    gt_r.t = t;
end


% Generate random world frame data
%   - Randomly generate polynomial for 
%     each state and its derivatives
%   - Generate a time series of discrete
%     measurements
%   - Evaluate the polynomial for each time
%     and return that data
%
% Returns:
%   wf_data = {t, pos, vel, acc, R, omega}
function wf_data = generate_world_frame(t)

    % polynomial parameters
    ord = 5;
    max_val = 20;


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

% Undo skew operator
function u = unskew(ux)
    u(3,1) = -ux(1,2);
    u(2,1) = ux(1,3);
    u(1,1) = -ux(2,3);
end

% Log mapping from SE(3) -> R^3
function w = Log(R)
    w = unskew(logm(R));
end