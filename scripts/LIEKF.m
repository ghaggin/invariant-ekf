classdef LIEKF < handle
%% Left-Invariant filter class, predicts next state, corrects prediction
    properties 
        mu;         %Pose Mean
        bias;       %Bias of gyro and accelerometer = [wb,ab]'; 
        Sigma;      %Pose Sigma
        A;          %Process model - Not currently being used
        cov_g;      %gyro noise
        cov_a;      %acc noise
        cov_gb;     %gyro bias
        cov_ab;     %acc bias
        V;          %observation noise of position
        Q;          %all covariance in process model
        %mu_cart
        %sigma_cart
    end
    methods
        function obj = LIEKF(R0, p0, v0)
            % Set the initial state
            if nargin == 0
                R0 = eye(3);
                p0 = zeros(3,1);
                v0 = zeros(3,1);
            end
            obj.mu = blkdiag(R0, eye(2));
            obj.mu(1:3,4) = v0;
            obj.mu(1:3,5) = p0;

            obj.Sigma = eye(15); %TBT
            obj.bias = zeros(6,1);
           
            obj.cov_g = eye(3); %TBT
            obj.cov_a = eye(3);
            obj.cov_gb = eye(3);
            obj.cov_ab = eye(3);
%             obj.V = diag([1,1,10]);
            obj.V =     [4.6778    1.9437    0.0858;
                         1.9437   11.5621    5.8445;
                         0.0858    5.8445   22.4051]*1000;
            obj.Q = blkdiag([
                obj.cov_g,zeros(3),zeros(3),zeros(3),zeros(3);
                zeros(3),obj.cov_a,zeros(3),zeros(3),zeros(3);
                zeros(3),zeros(3),eye(3),zeros(3),zeros(3);
                zeros(3),zeros(3),zeros(3),obj.cov_gb,zeros(3),;
                zeros(3),zeros(3),zeros(3),zeros(3),obj.cov_ab])*1;
            obj.A = @(wt,at) [
                -obj.skew(wt), zeros(3),  zeros(3), -eye(3), zeros(3); 
                -obj.skew(at), -obj.skew(wt), zeros(3), zeros(3), -eye(3);
                zeros(3), eye(3), -obj.skew(wt), zeros(3),zeros(3);
                zeros(3), zeros(3), zeros(3), zeros(3), zeros(3);
                zeros(3), zeros(3), zeros(3), zeros(3), zeros(3)
            ];
        end

        function [R, p, v] = getState(obj)
            R = obj.mu(1:3, 1:3);
            v = obj.mu(1:3, 4);
            p = obj.mu(1:3, 5);
        end
        
        function prediction(obj, w, a, dt)  %TBC bias
            skew = @(u) obj.skew(u);

            % Predicts position from gyro/accelerometer data
            gamma0 = @(phi) eye(3) + sin(norm(phi,2))/norm(phi,2) * skew(phi) ...
                + (1-cos(norm(phi,2)))/(norm(phi,2)^2) * (skew(phi))^2;

            gamma1 = @(phi) eye(3) + (1-cos(norm(phi,2)))/(norm(phi,2)^2) * (skew(phi)) ...
                + (norm(phi,2) - sin(norm(phi,2)))/(norm(phi,2)^3) * skew(phi)^2;

            gamma2 = @(phi) 0.5*eye(3) + (norm(phi,2) - sin(norm(phi,2)))/(norm(phi,2)^3) * skew(phi) ... 
                + (norm(phi,2)^2 + 2*cos(norm(phi,2)) - 2)/(2*(norm(phi,2)^4)) * skew(phi)^2;

            % Bias stuff?
            wb = obj.bias(1:3); %TBC
            ab = obj.bias(4:6);
            wt = w - wb;    %true noisy value
            at = a - ab;
            
            % Store state in convenient variables
            [R, p, v] = obj.getState();

            % Integrate the angular rates
            % This is equivalent to 
            %   R_k = R*expm(skew(w*dt))
            % only using the gamma function to 
            % construct the rotation matrix expm(skew(w*dt))
            Rk = R*gamma0(w*dt);

            % An accelerometer can't tell the difference between
            % the pull of a gravitational field in one direction
            % and an acceleration in the opposite direction.
            % Let g = [0;0;-9.81], then the measured accel a_m is
            %   a_m = R^T*(a_e - g)
            % where a_e is the actual acceleration in the earth frame
            % and R is the rotation of the body frame with respect to earth.
            % Solving for a_e (to integrate), we get
            %   a_e = R*a_m + g
            %
            % With the earth frame acceleration, integrate once for velocity
            % and a second time for position.  Formulas with the gamma function
            % from slides (I don't know the derivation but they appear to work)
            g = [0;0;-9.81];
            vk = v + R*gamma1(wt*dt)*at*dt + g*dt;
            pk = p + v*dt + R*gamma2(wt*dt)*at*(dt^2) + 0.5*g*(dt^2);
            phi = expm(obj.A(wt,at)*dt); 
            
            % Set the mean to the predicted value
            obj.mu = [
                Rk, vk, pk; 
                zeros(2,3), eye(2)
            ];
%             obj.bias = zeros(6,1);
            obj.Sigma = phi*obj.Sigma*(phi') + phi*obj.Q*(phi')*dt;
        end
                
        %GPS 3x1 is this in R^3 ECEF/NED/ENU??
        function correction(obj,GPS)
%             GPS(end) = 0;
            Y = [GPS;0;1];
            H = [zeros(3),zeros(3), eye(3), zeros(3), zeros(3)];
            
            [R, ~, ~] = obj.getState();
            
            N = R' * obj.V * R;
            
            S = H * obj.Sigma * H' + N;
            K = obj.Sigma * H' / S;     % Kalman gain
            K_X = K(1:9,:);
            K_B = K(10:15,:);
            PI = [eye(3), zeros(3,2)];
            nu = eye(5)/obj.mu * Y;        % Innovation
            delta_X = K_X * PI * nu;         
            delta_B = K_B * PI * nu;         
            xi = obj.makeTwist(delta_X);     % basically the obj.wedge operator for our 9-dimensional state space
            
            obj.mu = obj.mu * expm(xi);
            obj.bias = obj.bias + delta_B;
            obj.Sigma = (eye(15)- K * H) * obj.Sigma * (eye(15)- K * H)' + K * N * K';
        end
        
        function skew = skew(obj, u)
            skew = [
                0 -u(3) u(2)
                u(3) 0 -u(1)
                -u(2) u(1) 0
            ];  
        end
        
        function xi = makeTwist(obj, delta)
            % from vector 9x1 to se2(3) 5x5
            R_lie = obj.skew(delta(1:3));
            v_lie = [delta(4);delta(5);delta(6)];
            p_lie = [delta(7);delta(8);delta(9)];
            xi = [R_lie, v_lie, p_lie;...
                  zeros(1,3),  0,     0;...
                  zeros(1,3),  0,     0];
        end
    end
end