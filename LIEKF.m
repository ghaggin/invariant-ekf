classdef LIEKF < handle
%% Left-Invariant filter class, predicts next state, corrects prediction
    properties 
        mu;         %Pose Mean
        bias;       %Bias of gyro and accelerometer = [wb,ab]'; 
        Sigma;      %Pose Sigma
%         mu_pred;    %Mean after prediction step
%         bias_pred;  %Bias after prediction step
%         Sigma_pred; %Sigma after prediction step
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
        function obj = LIEKF
            R =     [0.4347   -0.7909   -0.4307
                    -0.6604    0.0452   -0.7495
                    0.6123    0.6102   -0.5027]; %???initial rotation matrix              
            obj.mu = blkdiag(R,eye(2));
            obj.Sigma = eye(15); %TBT
            obj.bias = zeros(6,1);
           
            obj.cov_g = eye(3); %TBT
            obj.cov_a = eye(3);
            obj.cov_gb = eye(3);
            obj.cov_ab = eye(3);
            obj.V = [4.6778    1.9437    1.3148 % Covariance of observation noise (see getZurichData.m)
                    1.9437   11.5621    6.9711
                    1.3148    6.9711   43.9883]; 
            obj.Q = blkdiag([obj.cov_g,zeros(3),zeros(3),zeros(3),zeros(3);
                             zeros(3),obj.cov_a,zeros(3),zeros(3),zeros(3);
                             zeros(3),zeros(3),eye(3),zeros(3),zeros(3);
                             zeros(3),zeros(3),zeros(3),obj.cov_gb,zeros(3),;
                             zeros(3),zeros(3),zeros(3),zeros(3),obj.cov_ab]);
                         
            obj.A = @(wt,at) [-obj.wedge(wt),   zeros(3),        zeros(3),      -eye(3),   zeros(3); 
                              -obj.wedge(at),   -obj.wedge(wt),  zeros(3),      zeros(3), -eye(3);
                               zeros(3),        eye(3),         -obj.wedge(wt), zeros(3),  zeros(3);
                               zeros(3),        zeros(3),        zeros(3),      zeros(3),  zeros(3);
                               zeros(3),        zeros(3),        zeros(3),      zeros(3),  zeros(3)];
        end
        
        function prediction(obj, w, a, dt)  %TBC bias
            % Predicts position from gyro/accelerometer data
            gamma0 = @(phi) eye(3) + sin(norm(phi,2))/norm(phi,2) * obj.wedge(phi) ...
                + (1-cos(norm(phi,2)))/(norm(phi,2)^2) * (obj.wedge(phi))^2;

            gamma1 = @(phi) eye(3) + (1-cos(norm(phi,2)))/(norm(phi,2)^2) * (obj.wedge(phi)) ...
                + (norm(phi,2) - sin(norm(phi,2)))/(norm(phi,2)^3) * obj.wedge(phi)^2;

            gamma2 = @(phi) 0.5*eye(3) + (norm(phi,2) - sin(norm(phi,2)))/(norm(phi,2)^3) * obj.wedge(phi) ... 
                + (norm(phi,2)^2 + 2*cos(norm(phi,2)) - 2)/(2*(norm(phi,2)^4)) * obj.wedge(phi)^2;

            wb = obj.bias(1:3); %TBC
            ab = obj.bias(4:6);
            wt = w - wb;    %true noisy value
            at = a - ab;
            
            R = obj.mu(1:3,1:3);
            v = obj.mu(1:3,4);
            p = obj.mu(1:3,5);
            
            Rk = R*gamma0(wt*dt);
            vk = v + R*gamma1(wt*dt)*at*dt + [0;0;9.81]*dt;
            pk = p + v*dt + R*gamma2(wt*dt)*at*(dt^2) + 0.5*[0;0;9.81]*(dt^2);
            
            % I think this should be A(w,a) not A(wt,at)
            phi = expm(obj.A(wt,at)*dt); 
            
            obj.mu = [Rk,vk,pk; 0,0,0,1,0; 0,0,0,0,1];
            obj.Sigma = phi*obj.Sigma*(phi') + phi*obj.Q*(phi')*dt;
           
        end
                
        function correction(obj,GPS)    %GPS 3X1
            Y = [GPS;0;1];
            H = [zeros(3),zeros(3), ones(3), zeros(3), zeros(3)];
            R = obj.mu(1:3,1:3);
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
            obj.bias = delta_B;
            obj.Sigma = (eye(15)- K * H) * obj.Sigma * (eye(15)- K * H)' + K * N * K';
        end
        
        function R = wedge(obj, v)
            G1 = [...
                0     0     1;
                0     0     0;
                0     0     0];
            G2 = [...
                0     0     0;
                0     0     1;
                0     0     0];
            G3 = [...
                0    -1     0;
                1     0     0;
                0     0     0];

            R = G1*v(1) + G2*v(2) + G3*v(3);   
        end
        
        function xi = makeTwist(obj, delta)
            % from vector 9x1 to se2(3) 5x5
            R_lie = obj.wedge(delta(1:3));
            v_lie = [delta(4);delta(5);delta(6)];
            p_lie = [delta(7);delta(8);delta(9)];
            xi = [R_lie, v_lie, p_lie;...
                  zeros(1,3),  0,     0;...
                  zeros(1,3),  0,     0];
        end
        
    end
end