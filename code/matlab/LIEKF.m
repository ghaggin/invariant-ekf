classdef LIEKF < handle
    properties 
        mu;         %Pose Mean
        Sigma;      %Pose Sigma
        gfun;       %IMU model function
        mu_pred;    %Mean after prediction step
        Sigma_pred; %Sigma after prediction step
        %A;          %Process model - Not currently being used
        %mu_cart
        %sigma_cart
    end
    methods
        function obj = LIEKF()
%             obj.gfun = @(R,v,p,w,a) [R*wedge(w), R*a + [0;0;9.81], v; zeros(2,5)];
            obj.mu = eye(5); %In SE2(3) this will be the identity 
%             obj.mu(3,5) = 4.6902;
            obj.Sigma = eye(9)*10; %Not sure how else to initialize covariance
%             obj.A = @(w,a) [-wedge(w), zeros(3),  zeros(3); 
%                            -wedge(a), -wedge(w), zeros(3);
%                             zeros(3), eye(3),   -wedge(w)]; 
        end
        
        function prediction(obj, w, a, dt)
            % Predicts position from gyro/IMU data
            gamma0 = @(phi) eye(3) + sin(norm(phi,2))/norm(phi,2) * wedge(phi) ...
                + (1-cos(norm(phi,2)))/(norm(phi,2)^2) * (wedge(phi))^2;

            gamma1 = @(phi) eye(3) + (1-cos(norm(phi,2)))/(norm(phi,2)^2) * (wedge(phi)) ...
                + (norm(phi,2) - sin(norm(phi,2)))/(norm(phi,2)^3) * wedge(phi)^2;

            gamma2 = @(phi) 0.5*eye(3) + (norm(phi,2) - sin(norm(phi,2)))/(norm(phi,2)^3) * wedge(phi) ... 
                + (norm(phi,2)^2 + 2*cos(norm(phi,2)) - 2)/(2*(norm(phi,2)^4)) * wedge(phi)^2;

            % There are more formal ways to do this, but I had trouble with
            % those methods. See slide 35 of the InEKF lecture.
            R = obj.mu(1:3,1:3);
            Rk = R*gamma0(w*dt);
            
            v = obj.mu(1:3,4);
            vk = v + R*gamma1(w*dt)*a*dt + [0;0;9.81]*dt;
            
            p = obj.mu(1:3,5);
            pk = p + v*dt + R*gamma2(w*dt)*a*(dt^2) + 0.5*[0;0;9.81]*(dt^2);
            
            obj.mu_pred = [Rk,vk,pk; 0,0,0,1,0; 0,0,0,0,1];
%             u = obj.gfun(R,v,p,w,a);
            
            phi = obj.makeTransition(w,a,dt); 
            obj.Sigma_pred = phi*obj.Sigma*(phi')+ diag([1,1,1,1,1,1,1,1,1])*0.01; %FIND actual noise covariance

        end
        
        function phi = makeTransition(obj,w,a,dt) %#ok<*INUSD>
            gamma0 = @(phi) eye(3) + sin(norm(phi,2))/norm(phi,2) * wedge(phi) ...
                + (1-cos(norm(phi,2)))/(norm(phi,2)^2) * (wedge(phi))^2;

            gamma1 = @(phi) eye(3) + (1-cos(norm(phi,2)))/(norm(phi,2)^2) * (wedge(phi)) ...
                + (norm(phi,2) - sin(norm(phi,2)))/(norm(phi,2)^3) * wedge(phi)^2;

            gamma2 = @(phi) 0.5*eye(3) + (norm(phi,2) - sin(norm(phi,2)))/(norm(phi,2)^3) * wedge(phi) ... 
                + (norm(phi,2)^2 + 2*cos(norm(phi,2)) - 2)/(2*(norm(phi,2)^4)) * wedge(phi)^2;


            phi11 = @(w) gamma0(w*dt)';
            phi21 = @(w,a) -gamma0(w*dt)'*wedge(gamma1(w*dt)*a)*dt;
            phi31 = @(w,a) -gamma0(w*dt)'*wedge(gamma2(w*dt)*a)*dt*dt;
            phi22 = phi11;
            phi32 = @(w) phi11(w)*dt;
            phi33 = phi11;
            
            phi =   [phi11(w),   zeros(3), zeros(3);
                     phi21(w,a), phi22(w), zeros(3);
                     phi31(w,a), phi32(w), phi33(w)];
        end
        
        function correction(obj,gps)
            H = [zeros(3),zeros(3),eye(3)]; % Linearization of observation (I think)
            V = [4.6778    1.9437    1.3148 % Covariance of observation noise (see getZurichData.m)
                1.9437   11.5621    6.9711
                1.3148    6.9711   43.9883]; 
%             V = eye(3)/1000000;
            R = obj.mu_pred(1:3,1:3);
            N = R\V/(R'); % I think this rotates our covariance to be in the same frame as our robot
            S = H*obj.Sigma_pred*(H') + N; % Covariance gain
            L = obj.Sigma_pred*(H')/S;     % Kalman gain
            
            nu = obj.mu\[gps,0,1]';        % Innovation
            delta = L * nu(1:3);           % Apply gain to our innovation
            xi = obj.makeTwist(delta);     % basically the wedge operator for our 9-dimensional state space
            
            obj.mu = obj.mu_pred*expm(xi);
            obj.Sigma = (eye(9)-L*H)*obj.Sigma_pred*((eye(9)-L*H)') + L*N*(L');
        end
        
        function xi = makeTwist(obj, delta)
            % raises our twist vector -> exp(xi) is in SE2(3)
            xi = [0,         -delta(3), delta(2),  delta(4), delta(7);
                  delta(3),  0,         -delta(1), delta(5), delta(8);
                  -delta(2), delta(1),  0,         delta(6), delta(9);
                  0,         0,         0,         0,        0;
                  0,         0,         0,         0,        0];
        end
        
    end
end